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
//  DriverUnitreeH1 public member functions
// #############################################

/**
 * @brief Construct a driver instance using the specified network interface and control mode.
 * @param network_interface Network interface used for Unitree DDS communication.
 * @param control_mode Active control mode. Supported values are
 *        "position_controlled", "velocity_controlled", and "torque_controlled".
 * @param ENTER_DAMPING_MODE_ON_DEINIT If true, damping mode is entered during deinitialization.
 * @param DUMMY_MODE If true, the driver starts in dummy mode.
 * @param shutdown_signaler Shared sas::ShutdownSignaler. Forwarded to driver components
 *        and polled every tick. Used to signal the driver to execute controlled shutdown.
 * @throws std::runtime_error if the supplied control mode string is invalid.
 */
DriverUnitreeH1::DriverUnitreeH1(std::string network_interface, 
                                 std::string control_mode, 
                                 bool ENTER_DAMPING_MODE_ON_DEINIT, 
                                 bool DUMMY_MODE, 
                                 const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler)
                                 : arm_sdk_(shutdown_signaler, DriverUnitreeArmSDK::ROBOT::H1, 0.02),
                                   loco_client_(shutdown_signaler, DriverUnitreeLocoClient::ROBOT::H1, 0.01),
                                   low_state_(shutdown_signaler, DriverUnitreeLowState::ROBOT::H1){

    common_construction_tasks_(network_interface, control_mode, ENTER_DAMPING_MODE_ON_DEINIT, DUMMY_MODE);
    
}

/**
 * @brief Construct a driver instance using the specified network interface and control mode.
 * @param network_interface Network interface used for Unitree DDS communication.
 * @param control_mode Active control mode. Supported values are
 *        "position_controlled", "velocity_controlled", and "torque_controlled".
 * @param shutdown_signaler Shared sas::ShutdownSignaler. Forwarded to driver components
 *        and polled every tick. Used to signal the driver to execute controlled shutdown.
 * @throws std::runtime_error if the supplied control mode string is invalid.
 */
DriverUnitreeH1::DriverUnitreeH1(std::string network_interface, 
                                 std::string control_mode, 
                                 const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler)
                                 : arm_sdk_(shutdown_signaler, DriverUnitreeArmSDK::ROBOT::H1, 0.02),
                                   loco_client_(shutdown_signaler, DriverUnitreeLocoClient::ROBOT::H1, 0.01),
                                   low_state_(shutdown_signaler, DriverUnitreeLowState::ROBOT::H1){

    common_construction_tasks_(network_interface, control_mode, false, false);
}

/**
 * @brief Construct a driver instance using the default position-control mode.
 * @param network_interface Network interface used for Unitree DDS communication.
 * @param shutdown_signaler Shared sas::ShutdownSignaler. Forwarded to driver components
 *        and polled every tick. Used to signal the driver to execute controlled shutdown.
 */
DriverUnitreeH1::DriverUnitreeH1(std::string network_interface, 
                                 const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler)
                                 : arm_sdk_(shutdown_signaler, DriverUnitreeArmSDK::ROBOT::H1, 0.02),
                                   loco_client_(shutdown_signaler, DriverUnitreeLocoClient::ROBOT::H1, 0.01),
                                   low_state_(shutdown_signaler, DriverUnitreeLowState::ROBOT::H1){

    common_construction_tasks_(network_interface, "position_controlled", false, false);
    
}

/**
 * @brief Perform the initialization steps shared by all constructor variants.
 * @param network_interface Network interface used for Unitree DDS communication.
 * @param control_mode Active control mode. Supported values are
 *        "position_controlled", "velocity_controlled", and "torque_controlled".
 * @param ENTER_DAMPING_MODE_ON_DEINIT If true, damping mode is entered during deinitialization.
 * @param DUMMY_MODE If true, the driver is initialized in passive dummy mode.
 * @throws std::runtime_error if the supplied control mode string is invalid.
 */
void DriverUnitreeH1::common_construction_tasks_(std::string network_interface, 
                                                std::string control_mode, 
                                                bool ENTER_DAMPING_MODE_ON_DEINIT, 
                                                bool DUMMY_MODE){


    // Process arguments
    network_interface_ = network_interface;
    enter_damping_mode_on_deinit_ = ENTER_DAMPING_MODE_ON_DEINIT;
    is_dummy_ = DUMMY_MODE;

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
 * @brief Connect to the Unitree H1 communication channels and locomotion client.
 *
 * This initializes the DDS channel factory, brings up the low-state observer,
 * connects the arm SDK, and starts the loco client used for torso and stand-height
 * commands.
 *
 * @throws std::runtime_error if the underlying subscriber or client initialization fails.
 */
void DriverUnitreeH1::connect(){

    std::cout << "Connecting..." << std::endl;
    std::this_thread::sleep_for(comms_timeout_chrono_sec_);

    std::cout << "    Opening communications channel..." << std::endl;
    // Initialize the robot communication system with the given network interface.
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
    std::cout << "        Done." << std::endl;

    // Start high level locomotion client.
    std::cout << "    Connecting to Low State Observer..." << std::endl;
    low_state_.connect();
    std::cout << "        Done." << std::endl;

    // Connect Arm SDK
    std::cout << "    Connecting to Arm SDK..." << std::endl;
    arm_sdk_.connect();
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
 * @brief Initialize the robot after communication has been established.
 *
 * This prepares the locomotion stack and arm-control interfaces for active motion
 * commands, and transitions the driver into the initialized state.
 *
 * @throws std::runtime_error if the driver has not connected before initialization,
 *         or if the low-level startup sequence fails.
 */
void DriverUnitreeH1::initialize(){
    std::cout << "Initializing..." << std::endl;
    std::this_thread::sleep_for(comms_timeout_chrono_sec_);

    if(current_status_!=STATUS::CONNECTED){
        throw std::runtime_error("[DriverUnitreeH1::initialize] Initialize called when robot is not properly connected!");
    }

    std::cout << "    Initializing Low State Observer..." << std::endl;
    low_state_.initialize();
    std::cout << "        Done." << std::endl;

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
        std::this_thread::sleep_for(comms_timeout_chrono_sec_);
    }
    std::cout << "        Done." << std::endl;

    std::cout << "Initialization complete." << std::endl;
    current_status_ = STATUS::INITIALIZED;
}

/**
 * @brief Deinitializes the robot safely.
 *        Stops locomotion, optionally enters damping mode (if set), and places the upper body into a safe state.
 *        This function must not throw exceptions because it may be invoked during shutdown.
 */
void DriverUnitreeH1::deinitialize(){
    // This function may be called by the SAS destructor; therefore, it must remain exception-safe.
    std::this_thread::sleep_for(comms_timeout_chrono_sec_);

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
    if(enter_damping_mode_on_deinit_){
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

        std::cout << "    Deinitializing Low State Observer..." << std::endl;
        low_state_.deinitialize();
        std::cout << "        Done." << std::endl;
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] [DriverUnitreeH1::deinitialize] Exception caught while deinitializing: "<<e.what()<<std::endl;
    }

    std::cout << "Deinitialization complete." << std::endl;
    current_status_ = STATUS::DEINITIALIZED;
}

/**
 * @brief Disconnect from the Unitree robot and release communication resources.
 *
 * This method closes the robot communication channels and releases the shared
 * channel factory resources. It is designed to remain exception-safe because it
 * will be called during teardown.
 */
void DriverUnitreeH1::disconnect(){
    // This function may be called by the SAS destructor; therefore, it must remain exception-safe.
    std::this_thread::sleep_for(comms_timeout_chrono_sec_);
    
    if(current_status_!=STATUS::DEINITIALIZED){
        std::cout << "[ERROR] [DriverUnitreeH1::disconnect] Disconnect called when robot is not properly deinitialized!"<<std::endl;
        return;
    }

    std::cout << "Disconnecting..." << std::endl;

    // Close the channels
    try{
        std::cout << "    Closing all comms channels..." << std::endl;

        arm_sdk_.disconnect();
        loco_client_.disconnect();
        low_state_.disconnect();

        unitree::robot::ChannelFactory::Instance()->Release();

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
 * @brief Return the current upper-body joint positions from the robot state.
 * @return A VectorXd containing the upper-body joint positions in radians.
 * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
 */
VectorXd DriverUnitreeH1::get_upper_body_joint_positions() {

    // Check the robot is initialized
    if (current_status_ != STATUS::INITIALIZED) {
        throw std::runtime_error(
            "[DriverUnitreeH1::get_upper_body_joint_positions] "
            "Function called when robot is not properly initialized!");
    }

    std::vector<double> left_arm_positions_rad = arm_sdk_.get_positions(DriverUnitreeArmSDK::LIMB::LEFT_ARM);
    std::vector<double> right_arm_positions_rad = arm_sdk_.get_positions(DriverUnitreeArmSDK::LIMB::RIGHT_ARM);
    std::vector<double> waist_positions_rad = arm_sdk_.get_positions(DriverUnitreeArmSDK::LIMB::WAIST);

    // Sanity-check dimensions
    if (left_arm_positions_rad.size() != 4) {
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_positions] Expected 4 left arm joints, got " +std::to_string(left_arm_positions_rad.size()));
    }

    if (right_arm_positions_rad.size() != 4) {
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_positions] Expected 4 right arm joints, got " +std::to_string(right_arm_positions_rad.size()));
    }

    if (waist_positions_rad.size() != 1) {
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_positions] Expected 1 waist joint, got " +std::to_string(waist_positions_rad.size()));
    }

    VectorXd current_overall_jpos_rad(9);
    current_overall_jpos_rad.segment<4>(0) = Eigen::Map<const Vector4d>(left_arm_positions_rad.data());
    current_overall_jpos_rad.segment<4>(4) = Eigen::Map<const Vector4d>(right_arm_positions_rad.data());
    current_overall_jpos_rad(8) = waist_positions_rad[0];

    return current_overall_jpos_rad;
}

/**
 * @brief Return the current upper-body joint velocities from the robot state.
 * @return A VectorXd containing the upper-body joint velocities in radians per second.
 * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
 */
VectorXd DriverUnitreeH1::get_upper_body_joint_velocities() const {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_velocities] Function called when robot is not properly initialized!");
    }

    const VectorXd left_arm_velocities_rps = low_state_.get_joint_velocities(DriverUnitreeLowState::LIMB::LEFT_ARM);
    const VectorXd right_arm_velocities_rps = low_state_.get_joint_velocities(DriverUnitreeLowState::LIMB::RIGHT_ARM);
    const VectorXd waist_velocities_rps = low_state_.get_joint_velocities(DriverUnitreeLowState::LIMB::TORSO);

    // Sanity-check dimensions
    if (left_arm_velocities_rps.size() != 4 || right_arm_velocities_rps.size() != 4 || waist_velocities_rps.size() != 1){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_velocities] Unexpected limb dimensions.");
    }

    VectorXd joint_velocities_rps(9);
    joint_velocities_rps << left_arm_velocities_rps,
                         right_arm_velocities_rps,
                         waist_velocities_rps;

    return joint_velocities_rps;
}

/**
 * @brief Return the estimated torques for the upper-body joints.
 * @return A VectorXd containing the upper-body joint torques in newton-metres.
 * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
 */
VectorXd DriverUnitreeH1::get_upper_body_joint_torques() const
{
    if (current_status_ != STATUS::INITIALIZED) {
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_torques] Function called when robot is not properly initialized!");
    }

    const VectorXd left_arm_torques_Nm = low_state_.get_joint_torques(DriverUnitreeLowState::LIMB::LEFT_ARM);
    const VectorXd right_arm_torques_Nm = low_state_.get_joint_torques(DriverUnitreeLowState::LIMB::RIGHT_ARM);
    const VectorXd waist_torques_Nm = low_state_.get_joint_torques(DriverUnitreeLowState::LIMB::TORSO);

    // Sanity-check dimensions
    if (left_arm_torques_Nm.size() != 4 || right_arm_torques_Nm.size() != 4 || waist_torques_Nm.size() != 1){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_torques] Unexpected limb dimensions.");
    }

    VectorXd joint_torques_Nm(9);
    joint_torques_Nm << left_arm_torques_Nm,
                         right_arm_torques_Nm,
                         waist_torques_Nm;

    return joint_torques_Nm;
}

/**
 * @brief Return the current casing temperatures for all joints.
 * @return A VectorXd containing the joint temperatures in degrees Celsius.
 * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
 */
VectorXd DriverUnitreeH1::get_joint_temperatures() const
{
    if (current_status_ != STATUS::INITIALIZED) {
        throw std::runtime_error("[DriverUnitreeH1::get_joint_temperatures] Function called when robot is not properly initialized!");
    }

    // If this is the dummy driver, then return all zeros.
    if(is_dummy_){
        return VectorXd::Zero(robot_joints_.size());
    }

    const VectorXd left_arm_temps_C = low_state_.get_joint_temperatures(DriverUnitreeLowState::LIMB::LEFT_ARM);
    const VectorXd right_arm_temps_C = low_state_.get_joint_temperatures(DriverUnitreeLowState::LIMB::RIGHT_ARM);
    const VectorXd waist_temps_C = low_state_.get_joint_temperatures(DriverUnitreeLowState::LIMB::TORSO);

    // Sanity-check dimensions
    if (left_arm_temps_C.size() != 4 || right_arm_temps_C.size() != 4 || waist_temps_C.size() != 1){
        throw std::runtime_error("[DriverUnitreeH1::get_joint_temperatures] Unexpected limb dimensions.");
    }

    VectorXd joint_temps_C(9);
    joint_temps_C << left_arm_temps_C,
                     right_arm_temps_C,
                     waist_temps_C;

    return joint_temps_C;

}

// --------------------------------------------
//  IMU getter functions
// --------------------------------------------

/**
 * @brief Return the current torso velocity estimate.
 * @return A 3-vector containing {vx, vy, omega} in the robot frame.
 * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
 */
VectorXd DriverUnitreeH1::get_torso_velocity() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_torso_velocity] Function called when robot is not properly initialized!");
    }
    
    std::cout << "DriverUnitreeH1::get_torso_velocity is not yet implemented." << std::endl;
    return VectorXd::Zero(3);
}

/**
 * @brief Return the most recent IMU sample from the low-state observer.
 * @return The latest IMU data packet, including quaternion orientation,
 *         angular velocity, linear acceleration, and Euler angles when available.
 * @throws std::runtime_error if the driver is not initialized.
 */
DriverUnitreeLowState::IMUData DriverUnitreeH1::get_IMU_data() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_IMU_data] Function called when robot is not properly initialized!");
    }

    if(!is_dummy_){
        return low_state_.get_imu_data();
    }
    else{
        DriverUnitreeLowState::IMUData spoof_IMU_data;
        
        spoof_IMU_data.accelerometer={0, 0, 0};
        spoof_IMU_data.gyroscope={0, 0, 0};
        spoof_IMU_data.quaternion={1, 0, 0, 0};
        spoof_IMU_data.rpy={0, 0, 0};
        spoof_IMU_data.valid=true;

        return spoof_IMU_data;
    }
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
 * @brief Return the current stand height as a percentage of the configured operating range.
 * @return The current stand-height percentage within the robot's operating range.
 */
float DriverUnitreeH1::get_stand_height_percent() const {
    
    // If this is the dummy driver then spoof the data
    if(is_dummy_){
        return 50;
    }
    
    float stand_height;
    stand_height = loco_client_.get_stand_height();
    float percent = (stand_height-0.6)/(0.2)*100;
    return(percent);
}

/**
 * @brief Set the robot stand height as a percentage of the configured operating range.
 * @param desired_height_percent Desired stand-height percentage in the operating range.
 */
void DriverUnitreeH1::set_stand_height_percent(const float desired_height_percent){
    float absolute = ((desired_height_percent/100)*0.2)+0.6;
    loco_client_.set_stand_height(absolute);
}

// --------------------------------------------
//  Setter functions
// --------------------------------------------

/**
 * @brief Change the active upper-body control mode at runtime.
 * @param new_mode Supported mode string: "position_controlled",
 *        "velocity_controlled", or "torque_controlled".
 * @throws std::runtime_error if the driver is not initialized or the mode string is invalid.
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
 * @brief Send position commands to all upper-body joints.
 * @param desired_joint_positions_rad Desired joint positions in radians.
 * @throws std::runtime_error if the driver is not initialized, the active mode is not
 *         position control, or the input size is incorrect.
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
 * @brief Send velocity commands to all upper-body joints.
 * @param desired_joint_velocities_rad_per_sec Desired joint velocities in radians per second.
 * @throws std::runtime_error if the driver is not initialized, the active mode is not
 *         velocity control, or the input size is incorrect.
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
 * @brief Send torque commands to all upper-body joints.
 * @param desired_joint_torques_Nm Desired joint torques in newton-metres.
 * @throws std::runtime_error if the driver is not initialized, the active mode is not
 *         torque control, or the input size is incorrect.
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
    throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] This function has not been implemented yet on this branch!");
}

/**
 * @brief Send a torso-velocity command to the locomotion client.
 * @param desired_torso_velocity_mps_radps 3-vector containing the desired torso motion
 *        in the robot frame as {vx, vy, omega}.
 * @throws std::runtime_error if the driver is not initialized or the input size is not 3.
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