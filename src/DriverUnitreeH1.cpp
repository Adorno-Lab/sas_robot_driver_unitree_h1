/*
# Copyright (c) 2026-2026 Adorno-Lab software developments
#
#    This file is part of sas_robot_driver_unitree_h1.
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This software is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with this software.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Daniel S. J. Derwent, email: daniel.derwent@manchester.ac.uk
#
# ################################################################
*/

#include <chrono>
#include <iostream>
#include <thread> 
#include <sstream> 
#include <map> 
#include <vector> 
#include <string> 
#include <array>

#include <unitree/robot/h1/loco/h1_loco_api.hpp>
#include <unitree/robot/h1/loco/h1_loco_client.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <Eigen/Core>
#include <dqrobotics/DQ.h>


#include "DriverUnitreeH1.hpp"
#include "ErrorParsing.hpp"

using namespace DQ_robotics;
using namespace Eigen;

// #############################################
//  Define Implementation Class (Impl)
// #############################################

class DriverUnitreeH1::Impl
{
public:

    // #############################################
    //  Impl data members
    // #############################################

    std::string network_interface_ = "Not Initialized";
    bool enter_damping_mode_on_deinit_ = false;
    bool is_dummy_ = false;

    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> upper_body_subscriber_;
    unitree_go::msg::dds_::LowState_ state_msg_;

    static constexpr float comms_timeout_sec_ = 1.0f;
    static constexpr std::chrono::duration<double> comms_timeout_chrono_sec_ {comms_timeout_sec_};
    
    // #############################################
    //  Impl member functions
    // #############################################

    /**
     * @brief Default implementation constructor.
     *        Creates an empty Impl object and leaves all handles uninitialized.
     */
    Impl() = default;

    /**
     * @brief Validate that the upper-body state subscriber is initialized and receiving packets.
     *        This check fails if the DDS channel is not connected or if the robot is not producing state updates within the expected window.
     * @param calling_function Name of the caller used to build a clear error message.
     * @throws std::runtime_error if the subscriber is missing or a valid state stream cannot be established.
     */
    void check_upper_body_subscriber_setup(std::string calling_function){

        // Skip this check if the driver is in dummy mode.
        if(is_dummy_){
            return;
        }

        // Check that the channel has been initialized (time is -1 if not)
        float result = upper_body_subscriber_->GetLastDataAvailableTime();
        if (result<0)
        {
            throw std::runtime_error("[" + calling_function + "] Upper body state subscriber not initialized!");
        }

        // Check that we're actually receiving messages (the robot should 
        // publish at a minimum frequency of 50 Hz if properly connected)
        int new_messages_received = 0;
        float previous_message_time = 0;
        float current_message_time = 0;
        const auto start_time = std::chrono::steady_clock::now();
        int target_number_of_messages = 5;
        // Loop until desired number of messages have been received (or timeout)
        while(new_messages_received<target_number_of_messages){

            // Get message time
            current_message_time = upper_body_subscriber_->GetLastDataAvailableTime();

            // Check if this is a new message, or the previous one again
            if(current_message_time>previous_message_time){
                previous_message_time = current_message_time;
                new_messages_received++;
            }

            // Check for timeout
            if (std::chrono::steady_clock::now() - start_time > comms_timeout_chrono_sec_*target_number_of_messages)
            {
                std::ostringstream timeout_str;
                timeout_str << std::fixed << std::setprecision(1)
                            << comms_timeout_sec_ * target_number_of_messages;

                throw std::runtime_error(
                    "[" + calling_function +
                    "] Failed to establish connection to robot state subscriber "
                    "(received " + std::to_string(new_messages_received) +
                    " messages over " + timeout_str.str() + " seconds)");
            }
        }
    }

    /**
     * @brief Verify that the state subscriber is still receiving updates within the configured timeout.
     *        This protects against stale robot state data and communication loss.
     * @throws std::runtime_error if the state subscriber has not reported a recent message within the timeout window.
     */
    void check_robot_still_connected(){  

        // Skip this check if the driver is in dummy mode.
        if(is_dummy_){
            return;
        }

        int64_t most_recent_message_time = upper_body_subscriber_->GetLastDataAvailableTime();  
        int64_t now = unitree::common::GetCurrentMonotonicTimeNanosecond();  
        double elapsed_sec = static_cast<double>(now - most_recent_message_time) / 1e9;  
    
        if (elapsed_sec > comms_timeout_sec_) {  // comms_timeout_sec_ as a plain double, in seconds  
            std::ostringstream oss;  
            oss << "[DriverUnitreeH1::Impl::check_robot_still_connected] Robot state subscriber timed out, is it still connected? ("  
                << std::fixed << std::setprecision(1) << elapsed_sec << " seconds since last message)";  
            throw std::runtime_error(oss.str());  
        }  
    }   

};

// #############################################
//  DriverUnitreeH1 public member functions
// #############################################

/**
 * @brief Construct a DriverUnitreeH1 object with explicit network interface and control mode.
 * @param network_interface The network interface to use for Unitree communication.
 * @param control_mode The requested control mode string: "position_controlled", "velocity_controlled", or "torque_controlled".
 * @param ENTER_DAMPING_MODE_ON_DEINIT True to enter damping mode automatically on deinit, false otherwise
 * @param DUMMY_MODE True if driver started in dummy mode, false otherwise
 * @throws runtime_error if the provided control_mode string is invalid.
 */
DriverUnitreeH1::DriverUnitreeH1(std::string network_interface, 
                                 std::string control_mode, 
                                 bool ENTER_DAMPING_MODE_ON_DEINIT, 
                                 bool DUMMY_MODE, 
                                 const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler)
                                 : arm_sdk_(shutdown_signaler, DriverUnitreeArmSDK::ROBOT::H1, 0.02),
                                   loco_client_(shutdown_signaler, DriverUnitreeLocoClient::ROBOT::H1, 0.01){

    common_construction_tasks(network_interface, control_mode, ENTER_DAMPING_MODE_ON_DEINIT, DUMMY_MODE);
    
}

/**
 * @brief Construct a DriverUnitreeH1 object with explicit network interface and control mode.
 * @param network_interface The network interface to use for Unitree communication.
 * @param control_mode The requested control mode string: "position_controlled", "velocity_controlled", or "torque_controlled".
 * @throws runtime_error if the provided control_mode string is invalid.
 */
DriverUnitreeH1::DriverUnitreeH1(std::string network_interface, 
                                 std::string control_mode, 
                                 const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler)
                                 : arm_sdk_(shutdown_signaler, DriverUnitreeArmSDK::ROBOT::H1, 0.02),
                                   loco_client_(shutdown_signaler, DriverUnitreeLocoClient::ROBOT::H1, 0.01){

    common_construction_tasks(network_interface, control_mode, false, false);
}

/**
 * @brief Construct a DriverUnitreeH1 object with a default position control mode.
 * @param network_interface The network interface to use for Unitree communication.
 */
DriverUnitreeH1::DriverUnitreeH1(std::string network_interface, 
                                 const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler)
                                 : arm_sdk_(shutdown_signaler, DriverUnitreeArmSDK::ROBOT::H1, 0.02),
                                   loco_client_(shutdown_signaler, DriverUnitreeLocoClient::ROBOT::H1, 0.01){

    common_construction_tasks(network_interface, "position_controlled", false, false);
    
}

/**
 * @brief Perform the tasks that are common to all constructor variants
 * @param network_interface The network interface to use for Unitree communication.
 * @param control_mode The requested control mode string: "position_controlled", "velocity_controlled", or "torque_controlled".
 * @param ENTER_DAMPING_MODE_ON_DEINIT True to enter damping mode automatically on deinit, false otherwise
 * @param DUMMY_MODE True if driver started in dummy mode, false otherwise
 * @throws runtime_error if the provided control_mode string is invalid.
 */
void DriverUnitreeH1::common_construction_tasks(std::string network_interface, 
                                                std::string control_mode, 
                                                bool ENTER_DAMPING_MODE_ON_DEINIT, 
                                                bool DUMMY_MODE){

    // Create implementation object
    impl_ = std::make_shared<DriverUnitreeH1::Impl>();

    // Process arguments
    impl_->network_interface_ = network_interface;
    impl_->enter_damping_mode_on_deinit_ = ENTER_DAMPING_MODE_ON_DEINIT;
    impl_->is_dummy_ = DUMMY_MODE;

    if(control_mode=="position_controlled"){
        current_mode_ = MODE::POSITION_CONTROLLED;
    }
    else if(control_mode=="velocity_controlled"){
        current_mode_ = MODE::VELOCITY_CONTROLLED;
    }
    else if(control_mode=="torque_controlled"){
        current_mode_ = MODE::TORQUE_CONTROLLED;
    }
    else{
        throw std::runtime_error("[DriverUnitreeH1::DriverUnitreeH1] Invalid control mode string passed to constructor: '"+control_mode+"'");
    }
    current_mode_string_ = control_mode;

}

/**
 * @brief Establishes communication with the Unitree H1 robot.
 *        Initializes the channel factory, starts the upper body publisher and subscriber,
 *        and starts the locomotion client.
 * @throws runtime_error if subscriber setup or locomotion client initialization fails.
 */
void DriverUnitreeH1::connect(){

    std::cout << "Connecting..." << std::endl;
    std::this_thread::sleep_for(impl_->comms_timeout_chrono_sec_);

    std::cout << "    Opening communications channel..." << std::endl;
    // Initialize the robot communication system with the given network interface.
    unitree::robot::ChannelFactory::Instance()->Init(0, impl_->network_interface_);
    std::cout << "        Done." << std::endl;

    // Connect Arm SDK
    std::cout << "    Connecting to Arm SDK..." << std::endl;
    arm_sdk_.connect();
    std::cout << "        Done." << std::endl;

    // Start low level subscriber
    std::cout << "    Starting robot state subscriber..." << std::endl;
    impl_->upper_body_subscriber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lf/lowstate"));
    impl_->upper_body_subscriber_->InitChannel([&](const void *msg) {
        auto s = ( const unitree_go::msg::dds_::LowState_* )msg;
        memcpy( &impl_->state_msg_, s, sizeof( unitree_go::msg::dds_::LowState_ ) );
        }, 1);

    // // Check the subscriber is working by reading the time that the last message was received
    impl_->check_upper_body_subscriber_setup("DriverUnitreeH1::connect");
    std::cout << "        Done." << std::endl;

    // Start high level locomotion client.
    std::cout << "    Connecting to Loco Client..." << std::endl;
    loco_client_.connect();
    // The old driver set a comms timeout here, not sure how to do that in the new driver.
    std::cout << "        Done." << std::endl;
    
    std::cout << "Connection complete." << std::endl;
    current_status_ = STATUS::CONNECTED;
}

/**
 * @brief Initializes the robot after connection has been established.
 *        Starts the locomotion client and brings upper body joints into a controlled state.
 * @throws runtime_error if called before connect() or if starting the locomotion client fails.
 */
void DriverUnitreeH1::initialize(){
    std::cout << "Initializing..." << std::endl;
    std::this_thread::sleep_for(impl_->comms_timeout_chrono_sec_);

    if(current_status_!=STATUS::CONNECTED){
        throw std::runtime_error("[DriverUnitreeH1::initialize] Initialize called when robot is not properly connected!");
    }

    std::cout << "    Initializing Loco Client..." << std::endl;
    loco_client_.initialize();
    loco_client_.set_fsm_id(FSM_ID::START);
    std::cout << "        Done." << std::endl;

    std::cout << "    Initializing Arm SDK..." << std::endl;
    arm_sdk_.initialize();
    arm_sdk_.enable_arm_control();
    // Block driver thread until arm control has been established.
    // This ensures that by the time commands are being issued the arms are ready to go.
    while(!arm_sdk_.is_arm_control_enabled()){
        std::this_thread::sleep_for(impl_->comms_timeout_chrono_sec_);
    }
    std::cout << "        Done." << std::endl;

    std::cout << "Initialization complete." << std::endl;
    current_status_ = STATUS::INITIALIZED;
}

/**
 * @brief Deinitializes the robot safely.
 *        Stops locomotion, optionally enters damping mode, and places the upper body into a safe state.
 *        This function must not throw exceptions because it may be invoked during shutdown.
 */
void DriverUnitreeH1::deinitialize(){
    // This function may be called by the SAS destructor; therefore, it must remain exception-safe.
    std::this_thread::sleep_for(impl_->comms_timeout_chrono_sec_);

    if(current_status_!=STATUS::INITIALIZED){
        std::cout << "[ERROR] [DriverUnitreeH1::initialize] Deinitialize called when robot is not properly initialized!"<<std::endl;
        return;
    }

    std::cout << "Deinitializing..." << std::endl;

    try{
        // Stop any ongoing motion
        std::cout << "    Stopping ongoing motion..." << std::endl;
        loco_client_.set_target_high_level_velocities({0, 0, 0});
        std::cout << "        Done."<<std::endl;
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] [DriverUnitreeH1::deinitialize] Exception caught while stopping ongoing motion: "<<e.what()<<std::endl;
    }

    // Put the robot into damping mode if requested
    if(impl_->enter_damping_mode_on_deinit_){
        try{
            std::cout << "    Entering damping mode..." << std::endl;
            loco_client_.set_fsm_id(FSM_ID::DAMP);
            std::cout << "        Done." << std::endl;
        }
        catch (const std::exception& e){
            std::cout << "[ERROR] [DriverUnitreeH1::deinitialize] Exception caught while entering damping mode: "<<e.what()<<std::endl;
        }
    }

    try{
        std::cout << "    Deinitializing Arm SDK..." << std::endl;
        arm_sdk_.deinitialize(); // Note that deinitialize is a blocking call that winds down the arm control weight 
        std::cout << "        Done." << std::endl;

        std::cout << "    Deinitializing Loco Client..." << std::endl;
        loco_client_.deinitialize(); // This call does not change the FSM ID, so the robot should remain standing
        std::cout << "        Done." << std::endl;
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] [DriverUnitreeH1::deinitialize] Exception caught while deinitializing Arm SDK: "<<e.what()<<std::endl;
    }

    std::cout << "Deinitialization complete." << std::endl;
    current_status_ = STATUS::DEINITIALIZED;
}

/**
 * @brief Disconnects from the Unitree robot and closes communication channels.
 *        Releases the publisher, subscriber, and channel factory resources.
 *        This function must not throw exceptions because it may be called from the destructor.
 */
void DriverUnitreeH1::disconnect(){
    // This function may be called by the SAS destructor; therefore, it must remain exception-safe.
    std::this_thread::sleep_for(impl_->comms_timeout_chrono_sec_);
    
    if(current_status_!=STATUS::DEINITIALIZED){
        std::cout << "[ERROR] [DriverUnitreeH1::disconnect] Disconnect called when robot is not properly deinitialized!"<<std::endl;
        return;
    }

    std::cout << "Disconnecting..." << std::endl;

    // Close the channels
    try{
        std::cout << "    Closing all comms channels..." << std::endl;
        impl_->upper_body_subscriber_->CloseChannel();
        unitree::robot::ChannelFactory::Instance()->Release();

        arm_sdk_.disconnect();
        loco_client_.disconnect();

        std::cout << "        Done." << std::endl;
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] [DriverUnitreeH1::disconnect] Exception caught during disconnection: "<<e.what()<<std::endl;
    }

    std::cout << "Disconnection complete." << std::endl;
    current_status_ = STATUS::DISCONNECTED;
}

// --------------------------------------------
//  Upper body getter functions
// --------------------------------------------

/**
 * @brief Returns the current upper body joint positions from the robot state.
 * @return A VectorXd of upper body joint positions in radians.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
VectorXd DriverUnitreeH1::get_upper_body_joint_positions() {

    // Check the robot is initialized
    if (current_status_ != STATUS::INITIALIZED) {
        throw std::runtime_error(
            "[DriverUnitreeH1::get_upper_body_joint_positions] "
            "Function called when robot is not properly initialized!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    std::vector<double> left_arm_positions_rad = arm_sdk_.get_positions(DriverUnitreeArmSDK::LIMB::LEFT_ARM);
    std::vector<double> right_arm_positions_rad = arm_sdk_.get_positions(DriverUnitreeArmSDK::LIMB::RIGHT_ARM);
    std::vector<double> waist_positions_rad = arm_sdk_.get_positions(DriverUnitreeArmSDK::LIMB::WAIST);

    // Sanity-check dimensions
    if (left_arm_positions_rad.size() != 4) {
        throw std::runtime_error("[get_upper_body_joint_positions] Expected 4 left arm joints, got " +std::to_string(left_arm_positions_rad.size()));
    }

    if (right_arm_positions_rad.size() != 4) {
        throw std::runtime_error("[get_upper_body_joint_positions] Expected 4 right arm joints, got " +std::to_string(right_arm_positions_rad.size()));
    }

    if (waist_positions_rad.size() != 1) {
        throw std::runtime_error("[get_upper_body_joint_positions] Expected 1 waist joint, got " +std::to_string(waist_positions_rad.size()));
    }

    VectorXd current_overall_jpos_rad(9);
    current_overall_jpos_rad.segment<4>(0) = Eigen::Map<const Vector4d>(left_arm_positions_rad.data());
    current_overall_jpos_rad.segment<4>(4) = Eigen::Map<const Vector4d>(right_arm_positions_rad.data());
    current_overall_jpos_rad(8) = waist_positions_rad[0];

    return current_overall_jpos_rad;
}

/**
 * @brief Returns the current upper body joint velocities from the robot state.
 * @return A VectorXd of upper body joint velocities in radians per second.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
VectorXd DriverUnitreeH1::get_upper_body_joint_velocities() const {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_velocities] Function called when robot is not properly initialized!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd current_jvel_rad_per_sec = VectorXd::Zero(upper_body_joints_.size());
    for (int i = 0; i < upper_body_joints_.size(); ++i) {
        current_jvel_rad_per_sec(i) = impl_->state_msg_.motor_state().at(upper_body_joints_.at(i)).dq();
    }
    return current_jvel_rad_per_sec;
}

/**
 * @brief Returns the estimated torques for all upper body joints.
 * @return A VectorXd of estimated joint torques in Newton-meters.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
VectorXd DriverUnitreeH1::get_upper_body_joint_torques() const {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_torques] Function called when robot is not properly initialized!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd current_jtorque_Nm = VectorXd::Zero(upper_body_joints_.size());
    for (int i = 0; i < upper_body_joints_.size(); ++i) {
        current_jtorque_Nm(i) = impl_->state_msg_.motor_state().at(upper_body_joints_.at(i)).tau_est();
    }
    return current_jtorque_Nm;
}

/**
 * @brief Returns the current casing temperatures for all upper body joints.
 * @return A VectorXd of joint temperatures in degrees Celsius.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
VectorXd DriverUnitreeH1::get_joint_temperatures() const {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_temperatures] Function called when robot is not properly initialized!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd current_j_casing_temp_C = VectorXd::Zero(robot_joints_.size());
    
    for (int i = 0; i < robot_joints_.size(); ++i) {
        current_j_casing_temp_C(i) = static_cast<double>(impl_->state_msg_.motor_state().at(robot_joints_.at(i)).temperature());
    }
    return current_j_casing_temp_C;
}

// --------------------------------------------
//  IMU getter functions
// --------------------------------------------

/**
 * @brief Returns the torso velocity from IMU or locomotion sensors.
 * @return A VectorXd of size 3 representing {vx, vy, omega}.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
VectorXd DriverUnitreeH1::get_torso_velocity() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_torso_velocity] Function called when robot is not properly initialized!");
    }
    
    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    std::cout << "DriverUnitreeH1::get_torso_velocity is not yet implemented." << std::endl;
    return VectorXd::Zero(3);
}

/**
 * @brief Returns the current IMU orientation as a quaternion.
 * @return A DQ object representing the IMU quaternion orientation.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
DQ DriverUnitreeH1::get_IMU_orientation() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_IMU_orientation] Function called when robot is not properly initialized!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    // If this is the dummy driver, then return 1.
    if (impl_->is_dummy_){
        DQ imu_quat(1, 0, 0, 0, 0, 0, 0, 0);
        return imu_quat.normalize();
    }

    // Otherwise, return the real IMU data
    VectorXd current_imu_orientation = VectorXd::Zero(4);

    for (int i = 0; i < current_imu_orientation.size(); ++i) {
        current_imu_orientation(i) = impl_->state_msg_.imu_state().quaternion()[i];
    }

    DQ imu_quat(current_imu_orientation);
    return imu_quat.normalize();
}

/**
 * @brief Returns the current IMU gyroscope measurements.
 * @return A VectorXd of size 3 containing gyroscope readings in radians per second.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
VectorXd DriverUnitreeH1::get_gyroscope_data() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_gyroscope_data] Function called when robot is not properly initialized!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd gyroscope_data = VectorXd::Zero(3);

    // If this is the dummy driver, then return 0.
    if (impl_->is_dummy_){
        return gyroscope_data;
    }
    
    // Otherwise read the real gyro
    for (int i = 0; i < gyroscope_data.size(); ++i) {
        gyroscope_data(i) = impl_->state_msg_.imu_state().gyroscope()[i];
    }

    return gyroscope_data;
}

/**
 * @brief Returns the current IMU accelerometer measurements.
 * @return A VectorXd of size 3 containing accelerometer readings in meters per second squared.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
VectorXd DriverUnitreeH1::get_accelerometer_data() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_accelerometer_data] Function called when robot is not properly initialized!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd accelerometer_data = VectorXd::Zero(3);

    // If this is the dummy driver, then return 0.
    if (impl_->is_dummy_){
        return accelerometer_data;
    }
    
    // Otherwise read the real accelerometer
    for (int i = 0; i < accelerometer_data.size(); ++i) {
        accelerometer_data(i) = impl_->state_msg_.imu_state().accelerometer()[i];
    }

    return accelerometer_data;
}

/**
 * @brief Returns the current IMU Euler angles.
 * @return A VectorXd of size 3 containing roll, pitch, and yaw in radians.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
VectorXd DriverUnitreeH1::get_Euler_angles() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_Euler_angles] Function called when robot is not properly initialized!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd Euler_angles = VectorXd::Zero(3);
    
    for (int i = 0; i < Euler_angles.size(); ++i) {
        Euler_angles(i) = impl_->state_msg_.imu_state().rpy()[i];
    }

    return Euler_angles;
}

/**
 * @brief Returns the current IMU temperature.
 * @return The IMU temperature in degrees Celsius.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
int DriverUnitreeH1::get_IMU_temperature() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_IMU_temperature] Function called when robot is not properly initialized!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();
    
    int IMU_temp = impl_->state_msg_.imu_state().temperature();

    return IMU_temp;
}

// --------------------------------------------
//  Battery getter functions
// --------------------------------------------

/**
 * @brief Read the battery state of charge.
 * @return Battery charge percentage.
 * @throws std::runtime_error if the current firmware does not provide a usable battery-state payload.
 */
int DriverUnitreeH1::get_battery_state_of_charge() const {
    
    // This function currently raises an exception because the H1 firmware does not appear to publish
    // a reliable state-of-charge value through the available telemetry path.
    throw std::runtime_error("[DriverUnitreeH1::get_battery_state_of_charge] This capability is not currently supported by the available firmware telemetry.");
}

/**
 * @brief Read the battery temperature measurements.
 * @return A 2-element VectorXd containing the battery temperatures in degrees Celsius.
 * @throws std::runtime_error if the current firmware does not provide a valid battery-temperature payload.
 */
VectorXd DriverUnitreeH1::get_battery_temperatures() const {
    
    // This function currently raises an exception because the H1 firmware does not appear to publish
    // a reliable battery-temperature payload through the available telemetry path.
    throw std::runtime_error("[DriverUnitreeH1::get_battery_temperatures] This capability is not currently supported by the available firmware telemetry.");
}

// --------------------------------------------
//  Misc
// --------------------------------------------

/**
 * @brief Returns the robot standing height as a percentage of the configured height range.
 * @return The standing height percentage in the range of the robot's stand height limits.
 */
float DriverUnitreeH1::get_stand_height_percent() const {
    float stand_height;
    stand_height = loco_client_.get_stand_height();
    float percent = (stand_height-0.6)/(0.2)*100;
    return(percent);
}

/**
 * @brief Sets the robot standing height as a percentage of its configured range.
 * @param desired_height_percent The desired height percentage to set, mapped into the robot's absolute stand height range.
 */
void DriverUnitreeH1::set_stand_height_percent(const float desired_height_percent){
    float absolute = ((desired_height_percent/100)*0.2)+0.6;
    loco_client_.set_stand_height(absolute);
}

// --------------------------------------------
//  Setter functions
// --------------------------------------------

/**
 * @brief Changes the current upper body control mode at runtime.
 * @param new_mode One of "position_controlled", "velocity_controlled", or "torque_controlled".
 * @throws runtime_error if called when the robot is not initialized or if the mode string is invalid.
 */
void DriverUnitreeH1::change_control_mode(const std::string& new_mode) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::change_control_mode] Function called when robot is not properly initialized!");
    }
    std::string old_mode = current_mode_string_;
    if(new_mode=="position_controlled"){
        current_mode_ = MODE::POSITION_CONTROLLED;
    }
    else if(new_mode=="velocity_controlled"){
        current_mode_ = MODE::VELOCITY_CONTROLLED;
    }
    else if(new_mode=="torque_controlled"){
        current_mode_ = MODE::TORQUE_CONTROLLED;
    }
    else{
        throw std::runtime_error("[DriverUnitreeH1::change_control_mode] Invalid control mode string passed to function: '"+new_mode+"'");
    }
    current_mode_string_ = new_mode;

    std::cout<<"[WARNING] CONTROL MODE CHANGED FROM '"<<old_mode<<"' TO '"<<new_mode<<"'"<<std::endl;

}

std::string DriverUnitreeH1::get_control_mode() {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_control_mode] Function called when robot is not properly initialized!");
    }

    return current_mode_string_;
}

/**
 * @brief Sends position commands to all upper body joints.
 * @param desired_joint_positions_rad A VectorXd of desired joint positions in radians.
 * @throws runtime_error if the robot is not initialized, if the robot is not in position control mode, or if the input size is incorrect.
 */
void DriverUnitreeH1::set_upper_body_joint_positions(const VectorXd& desired_joint_positions_rad) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_positions] Function called when robot is not properly initialized!");
    }

    if(current_mode_!=MODE::POSITION_CONTROLLED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_positions] Function called when robot is not in position control mode!");
    }

    if(desired_joint_positions_rad.size()!=upper_body_joints_.size()){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_positions] Input has incorrect size! (got "+std::to_string(desired_joint_positions_rad.size())+", expected "+std::to_string(upper_body_joints_.size())+")");
    }

    std::vector<double> left_arm_values(desired_joint_positions_rad.data(),desired_joint_positions_rad.data() + 4);
    std::vector<double> right_arm_values(desired_joint_positions_rad.data() + 4,desired_joint_positions_rad.data() + 8);
    std::vector<double> waist_values{desired_joint_positions_rad(8)};

    arm_sdk_.set_target_positions(DriverUnitreeArmSDK::LIMB::LEFT_ARM, left_arm_values);
    arm_sdk_.set_target_positions(DriverUnitreeArmSDK::LIMB::RIGHT_ARM, right_arm_values);
    arm_sdk_.set_target_positions(DriverUnitreeArmSDK::LIMB::WAIST, waist_values);
}

/**
 * @brief Sends velocity commands to all upper body joints.
 * @param desired_joint_velocities_rad_per_sec A VectorXd of desired joint velocities in radians per second.
 * @throws runtime_error if the robot is not initialized, if the robot is not in velocity control mode, or if the input size is incorrect.
 */
void DriverUnitreeH1::set_upper_body_joint_velocities(const VectorXd& desired_joint_velocities_rad_per_sec) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] Function called when robot is not properly initialized!");
    }

    if(current_mode_!=MODE::VELOCITY_CONTROLLED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] Function called when robot is not in velocity control mode!");
    }

    if(desired_joint_velocities_rad_per_sec.size()!=upper_body_joints_.size()){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] Input has incorrect size! (got "+std::to_string(desired_joint_velocities_rad_per_sec.size())+", expected "+std::to_string(upper_body_joints_.size())+")");
    }

    // As of 05/10/26, the unitree drivers submodule does not support velocity control of the arms.
    throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] This function has not been implemented yet on this branch!");
    
}

/**
 * @brief Sends torque commands to all upper body joints.
 * @param desired_joint_torques_Nm A VectorXd of desired joint torques in Newton-meters.
 * @throws runtime_error if the robot is not initialized, if the robot is not in torque control mode, or if the input size is incorrect.
 */
void DriverUnitreeH1::set_upper_body_joint_torques(const VectorXd& desired_joint_torques_Nm) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] Function called when robot is not properly initialized!");
    }

    if(current_mode_!=MODE::TORQUE_CONTROLLED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] Function called when robot is not in torque control mode!");
    }

    if(desired_joint_torques_Nm.size()!=upper_body_joints_.size()){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] Input has incorrect size! (got "+std::to_string(desired_joint_torques_Nm.size())+", expected "+std::to_string(upper_body_joints_.size())+")");
    }

    // As of 05/10/26, the unitree drivers submodule does not support velocity control of the arms.
    throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] This function has not been implemented yet on this branch!");
}

/**
 * @brief Sends torso velocity commands to the locomotion client.
 * @param desired_torso_velocity_mps_radps A VectorXd of size 3 containing desired {vx, vy, omega}.
 * @throws runtime_error if the robot is not initialized or if the input size is not 3.
 */
void DriverUnitreeH1::set_torso_velocity(const VectorXd& desired_torso_velocity_mps_radps) {

    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::set_torso_velocity] Function called when robot is not properly initialized!");
    }

    if(desired_torso_velocity_mps_radps.size()!=3){
        throw std::runtime_error("[DriverUnitreeH1::set_torso_velocity] Input has incorrect size! (got "+std::to_string(desired_torso_velocity_mps_radps.size())+", expected 3 -> {x, y, omega})");
    }

    float vx, vy, v_yaw;
    vx = desired_torso_velocity_mps_radps(0);
    vy = desired_torso_velocity_mps_radps(1);
    v_yaw = desired_torso_velocity_mps_radps(2);
    loco_client_.set_target_high_level_velocities({vx, vy, v_yaw});
}



