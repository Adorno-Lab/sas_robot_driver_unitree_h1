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

    std::string network_interface_ = "Not Initialised";
    bool is_dummy_robot_ = false;

    std::shared_ptr<unitree::robot::h1::LocoClient> locomotion_client_;

    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> upper_body_publisher_;
    unitree_go::msg::dds_::LowCmd_ cmd_msg_;

    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> upper_body_subscriber_;
    unitree_go::msg::dds_::LowState_ state_msg_;

    bool enter_damping_mode_on_deinit_ = false;

    static constexpr float comms_timeout_sec_ = 1.0f;
    static constexpr std::chrono::duration<double> comms_timeout_chrono_sec_ {comms_timeout_sec_};
    static constexpr float kp_ = 60.f;
    static constexpr float pos_cmd_kd_ = 1.5f;
    static constexpr float vel_cmd_kd_ = 10.f;
    static constexpr std::chrono::duration<double> weight_ramp_overall_duration_sec_{4.0};
    static constexpr float expected_firmware_update_period_sec_ = 0.02;
    static constexpr std::chrono::duration<double> expected_firmware_update_period_chrono_sec_ {expected_firmware_update_period_sec_};
    static constexpr float expected_movement_period_sec_ = expected_firmware_update_period_sec_*10;

    static constexpr float joint_limit_intervention_margin_rad_ = 0.2;

    static constexpr float global_joint_velocity_limit_radps_ = 1.0;
    
    static constexpr float global_joint_caution_factor_ = 0.15;

    std::unordered_map<int, float> lower_position_limits_rad_ = {
    {8,	 -0.43+global_joint_caution_factor_},
    {0,	 -0.43+global_joint_caution_factor_},
    {1,	 -3.14+global_joint_caution_factor_},
    {2,  -0.26+global_joint_caution_factor_},
    {11, -0.87+global_joint_caution_factor_},
    {7,	 -0.43+global_joint_caution_factor_},
    {3,	 -0.43+global_joint_caution_factor_},
    {4,	 -3.14+global_joint_caution_factor_},
    {5,	 -0.26+global_joint_caution_factor_},
    {10, -0.87+global_joint_caution_factor_},
    {6,	 -2.35+global_joint_caution_factor_},
    {12, -2.87+global_joint_caution_factor_},
    {13, -3.11+global_joint_caution_factor_},
    {14, -4.45+global_joint_caution_factor_},
    {15, -1.25+global_joint_caution_factor_},
    {16, -2.87+global_joint_caution_factor_},
    {17, -0.34+global_joint_caution_factor_},
    {18, -1.30+global_joint_caution_factor_},
    {19, -1.25+global_joint_caution_factor_},
    };

    std::unordered_map<int, float> upper_position_limits_rad_ = {
    {8,	 0.43-global_joint_caution_factor_},
    {0,	 0.43-global_joint_caution_factor_},
    {1,	 2.53-global_joint_caution_factor_},
    {2,	 2.05-global_joint_caution_factor_},
    {11, 0.52-global_joint_caution_factor_},
    {7,	 0.43-global_joint_caution_factor_},
    {3,	 0.43-global_joint_caution_factor_},
    {4,	 2.53-global_joint_caution_factor_},
    {5,	 2.05-global_joint_caution_factor_},
    {10, 0.52-global_joint_caution_factor_},
    {6,	 2.35-global_joint_caution_factor_},
    {12, 2.87-global_joint_caution_factor_},
    {13, 0.34-global_joint_caution_factor_},
    {14, 1.30-global_joint_caution_factor_},
    {15, 2.61-global_joint_caution_factor_},
    {16, 2.87-global_joint_caution_factor_},
    {17, 3.11-global_joint_caution_factor_},
    {18, 4.45-global_joint_caution_factor_},
    {19, 2.61-global_joint_caution_factor_},
    };

    // #############################################
    //  Impl member functions
    // #############################################

    /**
     * @brief Default implementation constructor.
     *        Creates an empty Impl object and leaves all handles uninitialised.
     */
    Impl() = default;

    /**
     * @brief DriverUnitreeB1::Impl::set_joint_position_command helper function for position commands.
     *        Called in a few places when joint positions are being written to the motors. Centralises
     *        the logic for this so it can be changed in only one place.
     * @param joint_id The id of the joint to be written to in Unitree SDK terms.
     * @param target_position_rad The joint position in radians.
     */
    void set_joint_position_command(int joint_id, float target_position_rad)
    {
        // If the position target is changing too quickly, then set a new target that will respect the
        // velocity limit.
        auto current_position = state_msg_.motor_state().at(joint_id).q();
        auto estimated_speed = (target_position_rad - current_position)/expected_movement_period_sec_;
        if(estimated_speed>global_joint_velocity_limit_radps_){
            target_position_rad = current_position + global_joint_velocity_limit_radps_ * expected_movement_period_sec_;
        }
        else if(estimated_speed<-global_joint_velocity_limit_radps_){
            target_position_rad = current_position - global_joint_velocity_limit_radps_ * expected_movement_period_sec_;
        }
        
        auto &cmd = cmd_msg_.motor_cmd().at(joint_id);

        // Motors are torque controlled using the eqn:
        // Torque = kp * (q_des - q_curr) + kd * (dq_des - dq_curr) + tau_ff
        // So, here we set dq_des as 0, so there is some damping proportional to the speed.
        cmd.q(std::clamp(target_position_rad, lower_position_limits_rad_.at(joint_id), upper_position_limits_rad_.at(joint_id)));
        cmd.dq(0.0);
        cmd.kp(kp_);
        cmd.kd(pos_cmd_kd_);
        cmd.tau(0);
    }

    /**
     * @brief DriverUnitreeB1::Impl::set_joint_velocity_command helper function for velocity commands. 
     *        Called in a few places when joint velocities are being written to the motors. Centralises 
     *        the logic for this so it can be changed in only one place.
     * @param joint_id The id of the joint to be written to in Unitree SDK terms.
     * @param target_velocity_rad_per_sec The joint velocity in radians per second
     */
    void set_joint_velocity_command(int joint_id, float target_velocity_rad_per_sec)
    {
        target_velocity_rad_per_sec = scale_command_based_on_joint_position(target_velocity_rad_per_sec, joint_id);     
        
        auto &cmd = cmd_msg_.motor_cmd().at(joint_id);
        // Motors are torque controlled using the eqn:
        // Torque = kp * (q_des - q_curr) + kd * (dq_des - dq_curr) + tau_ff
        // So here we set kp=0, to eliminate the position term.
        // Note that velocity control is somewhat inaccurate at low speeds, because
        // kp=0 means the robot has no position-holding stiffness, and the weight of 
        // the joints generates a torque has a significant impact. Larger gains might
        // help, but experiments show that kd values above 10 result in a nasty grinding
        // sound from the motors, which probably isn't good.
        cmd.q(0);
        cmd.dq(std::clamp(target_velocity_rad_per_sec, -global_joint_velocity_limit_radps_, global_joint_velocity_limit_radps_));
        cmd.kp(0);
        cmd.kd(vel_cmd_kd_); 
        cmd.tau(0);
    }

    /**
     * @brief DriverUnitreeB1::Impl::set_joint_torque_command helper function for torque commands. 
     *        Called in a few places when joint torques are being written to the motors. Centralises 
     *        the logic for this so it can be changed in only one place.
     * @param joint_id The id of the joint to be written to in Unitree SDK terms.
     * @param target_torque_Nm The joint torque in Newton metres
     */
    void set_joint_torque_command(int joint_id, float target_torque_Nm)
    {
        target_torque_Nm = scale_command_based_on_joint_position(target_torque_Nm, joint_id);

        auto &cmd = cmd_msg_.motor_cmd().at(joint_id);
        // Motors are torque controlled using the eqn:
        // Torque = kp * (q_des - q_curr) + kd * (dq_des - dq_curr) + tau_ff
        // So here we set kp==kd=0, to eliminate the position and velocity terms.
        // and control things via the feed-forward torques directly
        cmd.q(0);
        cmd.dq(0);
        cmd.kp(0);
        cmd.kd(0); 
        cmd.tau(target_torque_Nm);
    }

    /**
     * @brief Scales a joint command based on the joint's proximity to its position limits.
     *        As the joint approaches its upper or lower limit, the command magnitude is reduced
     *        to avoid hitting the hard limit too quickly.
     * @param original_command The original command value, either velocity or torque.
     * @param joint_id The Unitree joint index to evaluate.
     * @return The scaled command that respects joint limit proximity.
     */
    float scale_command_based_on_joint_position(float original_command, int joint_id){
        auto current_position = state_msg_.motor_state().at(joint_id).q();
        if (original_command > 0.0)
        {
            double dist = upper_position_limits_rad_.at(joint_id) - current_position;
            if (dist < joint_limit_intervention_margin_rad_){
                original_command *= std::max(0.0, dist / joint_limit_intervention_margin_rad_);
            }
        }
        else
        {
            double dist = current_position - lower_position_limits_rad_.at(joint_id);
            if (dist < joint_limit_intervention_margin_rad_){
                original_command *= std::max(0.0, dist / joint_limit_intervention_margin_rad_);
            }
        }
        return (original_command);
    }

    /**
     * @brief DriverUnitreeB1::Impl::send_upper_body_control_message helper function for the publisher.
     *        Sends a message to the upper_body_publisher_ and throws a runtime_error if the write operation fails.
     *        Note that this function succeeding is not evidence that the robot received the message, only that the
     *        message was written to the publisher. There may be other problems that prevent it from receiving that
     *        message (e.g., if the robot doesn't subscribe to the topic, or isn't powered on). A failure of this
     *        function usually indicates that something is wrong with the upper_body_publisher_ itself.
     * @param calling_function The function calling send_upper_body_control_message, used for informative runtime errors.
     * @throws runtime_error if the publisher write fails.
     */
    void send_upper_body_control_message(std::string calling_function)
    {
        if (!upper_body_publisher_->Write(cmd_msg_))
        {
            throw std::runtime_error("[" + calling_function + "] Upper body control message failed to send!");
        }
    }

    /**
     * @brief DriverUnitreeB1::Impl::check_upper_body_subscriber_setup helper function for the subscriber.
     *        Checks recent messages from the subscriber and throws a runtime_error if the channel is initialised
     *        incorrectly, or if messages are not received within the expected window (which suggests that the robot
     *        is not connected to the channel).
     * @param calling_function The function calling check_upper_body_subscriber_setup, used for informative runtime errors.
     * @throws runtime_error if the subscriber is not initialized or if the robot state messages are not received in time.
     */
    void check_upper_body_subscriber_setup(std::string calling_function){

        // Skip this check if the driver is in dummy mode.
        if(is_dummy_robot_){
            return;
        }

        // Check that the channel has been initialised (time is -1 if not)
        float result = upper_body_subscriber_->GetLastDataAvailableTime();
        if (result<0)
        {
            throw std::runtime_error("[" + calling_function + "] Upper body state subscriber not initialised!");
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
     * @brief DriverUnitreeB1::Impl::set_joint_damping_mode_command helper function for damping mode.
     *        Configures a motor command so the joint is softly damped without enforcing a position target.
     * @param joint_id The id of the joint to be written to in Unitree SDK terms.
     */
    void set_joint_damping_mode_command(int joint_id){
        auto &cmd = cmd_msg_.motor_cmd().at(joint_id);

        // Motors are torque controlled using the eqn:
        // Torque = kp * (q_des - q_curr) + kd * (dq_des - dq_curr) + tau_ff
        // So, here we set dq_des as 0, so there is some damping proportional to the speed, but
        // also kp is zero, so we dont actually care what the position is. The effect is "damping
        // mode" but only for the upper body.
        cmd.q(0);
        cmd.dq(0.0);
        cmd.kp(0.0);
        cmd.kd(pos_cmd_kd_);
        cmd.tau(0);
    }

    /**
     * @brief DriverUnitreeB1::Impl::send_lower_body_control_message helper function for the locomotion client.
     *        Sends commands to the high-level locomotion client and reports any non-zero return codes.
     * @param message The command name to invoke on the locomotion client.
     * @param calling_function Name of the caller for informative error messages.
     * @param args Optional numeric arguments required by the selected command.
     * @return The float return value produced by some locomotion client commands, or 0.0 if none.
     * @throws runtime_error if the command name is unrecognized or if the locomotion client returns an error code.
     */
    float send_lower_body_control_message(std::string message, std::string calling_function, std::vector<float> args){
        int32_t loco_client_return_code = 0;
        float this_function_return_value = 0.0f;

        // Skip this check if the driver is in dummy mode.
        if(is_dummy_robot_){
            return this_function_return_value;
        }

        if (message == "Init") {
            locomotion_client_->Init(); // initialize the connection
        }
        else if (message == "SetTimeout") {
            locomotion_client_->SetTimeout(args[0]); // initialize the connection
        }
        else if (message == "Start") {
            loco_client_return_code = locomotion_client_->Start(); // Start the client running
        }
        else if (message == "StopMove") {
            loco_client_return_code = locomotion_client_->StopMove(); // Stop the robot if its currently moving
        }
        else if (message == "Damp") {
            loco_client_return_code = locomotion_client_->Damp(); // Enter damping mode
        }
        else if (message == "GetStandHeight") {
            loco_client_return_code = locomotion_client_->GetStandHeight(this_function_return_value); // Return the standing height
        }
        else if (message == "SetStandHeight") {
            loco_client_return_code = locomotion_client_->SetStandHeight(args[0]); // Set the standing height
        }
        else if (message == "SetVelocity") {
            loco_client_return_code = locomotion_client_->SetVelocity(args[0], args[1], args[2], args[3]); // Set the torso velocity
        }
        else if (message == "StandUp") {
            loco_client_return_code = locomotion_client_->StandUp(); // Come out of damping mode
        }
        else {
            throw std::runtime_error(
                        "[" + calling_function +
                        "] Message passed to high level locomotion client for which no case was defined: " + message);
        }

        // return code should always be 0. If it isn't 0, it means there was an error signal returned.
        if(loco_client_return_code!=0){
            const auto error_it = get_readable_error.find(loco_client_return_code);
            const std::string error_message = (error_it != get_readable_error.end())
                ? error_it->second
                : "unknown error";

            throw std::runtime_error(
                    "[" + calling_function +
                    "] Error message returned by high level locomotion client: '" + error_message + "' in response to command: '" + message + "'");
        }

        return this_function_return_value;
    }

    /**
     * @brief DriverUnitreeB1::Impl::check_robot_still_connected verifies the state subscriber is still receiving messages.
     *        Throws a runtime_error when the latest message is older than the configured communication timeout.
     * @throws runtime_error if the state subscriber has not received a recent message within the configured timeout.
     */
    void check_robot_still_connected(){  

        // Skip this check if the driver is in dummy mode.
        if(is_dummy_robot_){
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
 * @throws runtime_error if the provided control_mode string is invalid.
 */
DriverUnitreeH1::DriverUnitreeH1(std::string network_interface, std::string control_mode, bool ENTER_DAMPING_MODE_ON_DEINIT){

    // Create implementation object
    impl_ = std::make_shared<DriverUnitreeH1::Impl>();

    // Process arguments
    impl_->network_interface_ = network_interface;
    impl_->enter_damping_mode_on_deinit_ = ENTER_DAMPING_MODE_ON_DEINIT;

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
    
}

/**
 * @brief Construct a DriverUnitreeH1 object with explicit network interface and control mode.
 * @param network_interface The network interface to use for Unitree communication.
 * @param control_mode The requested control mode string: "position_controlled", "velocity_controlled", or "torque_controlled".
 * @throws runtime_error if the provided control_mode string is invalid.
 */
DriverUnitreeH1::DriverUnitreeH1(std::string network_interface, std::string control_mode){

    // Create implementation object
    impl_ = std::make_shared<DriverUnitreeH1::Impl>();

    // Process arguments
    impl_->network_interface_ = network_interface;

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
    
}

/**
 * @brief Construct a DriverUnitreeH1 object with a default position control mode.
 * @param network_interface The network interface to use for Unitree communication.
 */
DriverUnitreeH1::DriverUnitreeH1(std::string network_interface){

    // Create implementation object
    impl_ = std::make_shared<DriverUnitreeH1::Impl>();

    // Process arguments
    impl_->network_interface_ = network_interface;

    // Enact defaults
    current_mode_ = MODE::POSITION_CONTROLLED;
    
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

    // Start low level publisher
    std::cout << "    Starting robot command publisher..." << std::endl;
    impl_->upper_body_publisher_.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>("rt/arm_sdk"));
    impl_->upper_body_publisher_->InitChannel();
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
    std::cout << "    Connecting to locomotion server..." << std::endl;
    impl_->locomotion_client_ = std::make_shared<unitree::robot::h1::LocoClient>();
    // impl_->locomotion_client_->Init(); // initialize the connection
    //     impl_->locomotion_client_->SetTimeout(impl_->comms_timeout_sec_); // set a 10 second timeout
    impl_->send_lower_body_control_message("Init", "DriverUnitreeH1::connect", {}); // initialize the connection
    impl_->send_lower_body_control_message("SetTimeout", "DriverUnitreeH1::connect", {impl_->comms_timeout_sec_});// set a 10 second timeout
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
    std::cout << "Initialising..." << std::endl;
    std::this_thread::sleep_for(impl_->comms_timeout_chrono_sec_);

    if(current_status_!=STATUS::CONNECTED){
        throw std::runtime_error("[DriverUnitreeH1::initialize] Initialize called when robot is not properly connected!");
    }

    std::cout << "    Starting locomotion client..." << std::endl;
    impl_->send_lower_body_control_message("Start", "DriverUnitreeH1::initialize", {});
    std::cout << "        Done." << std::endl;


    std::cout << "    Initialising upper body joints..." << std::endl;
    safely_start_upper_body_joints_();
    std::cout << "        Done." << std::endl;

    std::cout << "Initialisation complete." << std::endl;
    current_status_ = STATUS::INITIALIZED;
}

/**
 * @brief Deinitializes the robot safely.
 *        Stops locomotion, optionally enters leg damping mode, and stops upper body joints.
 *        This function must not throw exceptions because it may be called from the destructor.
 */
void DriverUnitreeH1::deinitialize(){
    // IMPORTANT NOTE: This function is called by the SAS destructor. Therfore, for safety reasons, it must not
    // generate any exceptions.
    std::this_thread::sleep_for(impl_->comms_timeout_chrono_sec_);

    if(current_status_!=STATUS::INITIALIZED){
        std::cout << "[ERROR] [DriverUnitreeH1::initialize] Deinitialize called when robot is not properly initialised!"<<std::endl;
        return;
    }

    std::cout << "Deinitialising..." << std::endl;

    try{
        // Stop any ongoing motion
        std::cout << "    Stopping locomotion server..." << std::endl;
        // impl_->locomotion_client_->StopMove();
        impl_->send_lower_body_control_message("StopMove", "DriverUnitreeH1::deinitialize", {});
        std::cout << "        Done."<<std::endl;
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] [DriverUnitreeH1::deinitialize] Exception caught while stopping locomotion server: "<<e.what()<<std::endl;
    }

    // Put the robot into damping mode if requested
    if(impl_->enter_damping_mode_on_deinit_){
        try{
            std::cout << "    Entering damping mode..." << std::endl;
            // impl_->locomotion_client_->Damp();
            impl_->send_lower_body_control_message("Damp", "DriverUnitreeH1::deinitialize", {});
            std::cout << "        Done." << std::endl;
        }
        catch (const std::exception& e){
            std::cout << "[ERROR] [DriverUnitreeH1::deinitialize] Exception caught while entering damping mode: "<<e.what()<<std::endl;
        }
    }
    else{
        try{
            std::cout << "    Deinitialising upper body joints..." << std::endl;
            safely_stop_upper_body_joints_();
            std::cout << "        Done." << std::endl;
        }
        catch (const std::exception& e){
            std::cout << "[ERROR] [DriverUnitreeH1::deinitialize] Exception caught while deinitialising upper body joints: "<<e.what()<<std::endl;
        }
    }

    std::cout << "Deinitialisation complete." << std::endl;
    current_status_ = STATUS::DEINITIALIZED;
}

/**
 * @brief Disconnects from the Unitree robot and closes communication channels.
 *        Releases the publisher, subscriber, and channel factory resources.
 *        This function must not throw exceptions because it may be called from the destructor.
 */
void DriverUnitreeH1::disconnect(){
    // IMPORTANT NOTE: This function is called by the SAS destructor. Therfore, for safety reasons, it must not
    // generate any exceptions.
    std::this_thread::sleep_for(impl_->comms_timeout_chrono_sec_);
    
    if(current_status_!=STATUS::DEINITIALIZED){
        std::cout << "[ERROR] [DriverUnitreeH1::disconnect] Disconnect called when robot is not properly deinitialised!"<<std::endl;
        return;
    }

    std::cout << "Disconnecting..." << std::endl;

    // Close the channels
    try{
        std::cout << "    Closing all comms channels..." << std::endl;
        impl_->upper_body_publisher_->CloseChannel();
        impl_->upper_body_subscriber_->CloseChannel();
        unitree::robot::ChannelFactory::Instance()->Release();
        std::cout << "        Done." << std::endl;
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] [DriverUnitreeH1::disconnect] Exception caught while closing upper body coms channels: "<<e.what()<<std::endl;
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
VectorXd DriverUnitreeH1::get_upper_body_joint_positions() const {
    
    // Check the robot is initialised
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_positions] Function called when robot is not properly initialised!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd current_jpos_rad = VectorXd::Zero(upper_body_joints_.size());
    for (int i = 0; i < upper_body_joints_.size(); ++i) {
        current_jpos_rad(i) = impl_->state_msg_.motor_state().at(upper_body_joints_.at(i)).q();
    }
    return current_jpos_rad;
}

/**
 * @brief Returns the current upper body joint velocities from the robot state.
 * @return A VectorXd of upper body joint velocities in radians per second.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
VectorXd DriverUnitreeH1::get_upper_body_joint_velocities() const {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_velocities] Function called when robot is not properly initialised!");
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
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_torques] Function called when robot is not properly initialised!");
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
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_temperatures] Function called when robot is not properly initialised!");
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
            throw std::runtime_error("[DriverUnitreeH1::get_torso_velocity] Function called when robot is not properly initialised!");
    }
    
    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    std::cout<<"DriverUnitreeH1::get_torso_velocity is not yet implemented"<<std::endl;
    return VectorXd::Zero(3);
}

/**
 * @brief Returns the current IMU orientation as a quaternion.
 * @return A DQ object representing the IMU quaternion orientation.
 * @throws runtime_error if the robot is not initialized or if the state subscriber has timed out.
 */
DQ DriverUnitreeH1::get_IMU_orientation() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_IMU_orientation] Function called when robot is not properly initialised!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

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
            throw std::runtime_error("[DriverUnitreeH1::get_gyroscope_data] Function called when robot is not properly initialised!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd gyroscope_data = VectorXd::Zero(3);
    
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
            throw std::runtime_error("[DriverUnitreeH1::get_accelerometer_data] Function called when robot is not properly initialised!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd accelerometer_data = VectorXd::Zero(3);
    
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
            throw std::runtime_error("[DriverUnitreeH1::get_Euler_angles] Function called when robot is not properly initialised!");
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
            throw std::runtime_error("[DriverUnitreeH1::get_IMU_temperature] Function called when robot is not properly initialised!");
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
 * @brief Returns the battery state of charge.
 * @return The battery state of charge in percent.
 * @throws runtime_error because this function is not ready to use.
 */
int DriverUnitreeH1::get_battery_state_of_charge() const {
    
    // This function always returns 0 for the state of charge. The reason is not clear, but it is possible that the
    // H1's firmware is just not publishing that information. Resolving this will likely mean a discussion with
    // autodiscovery.
    throw std::runtime_error("[DriverUnitreeH1::get_battery_state_of_charge] This function is not ready to use!");
    

    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_battery_state_of_charge] Function called when robot is not properly initialised!");
    }
    
    
    int battery_soc_percent = static_cast<int>(impl_->state_msg_.bms_state().soc());
    return battery_soc_percent;
}

/**
 * @brief Returns the battery temperatures.
 * @return A VectorXd of size 2 containing battery temperature readings in degrees Celsius.
 * @throws runtime_error because this function is not ready to use.
 */
VectorXd DriverUnitreeH1::get_battery_temperatures() const {
    
    // This function always returns [0, 0] for the temperatures. The reason is not clear, but it is possible that the
    // H1's firmware is just not publishing that information. Resolving this will likely mean a discussion with
    // autodiscovery.
    throw std::runtime_error("[DriverUnitreeH1::get_battery_temperatures] This function is not ready to use!");

    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_battery_temperatures] Function called when robot is not properly initialised!");
    }

    VectorXd current_battery_temp_C = VectorXd::Zero(2);
    
    for (int i = 0; i < 2; ++i) {
        current_battery_temp_C(i) = impl_->state_msg_.bms_state().bq_ntc()[i];
    }
    return current_battery_temp_C;
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
    // impl_->locomotion_client_->GetStandHeight(stand_height);
    stand_height = impl_->send_lower_body_control_message("GetStandHeight","DriverUnitreeH1::get_stand_height_percent",{});
    float percent = (stand_height-0.6)/(0.2)*100;
    return(percent);
}

/**
 * @brief Sets the robot standing height as a percentage of its configured range.
 * @param desired_height_percent The desired height percentage to set, mapped into the robot's absolute stand height range.
 */
void DriverUnitreeH1::set_stand_height_percent(const float desired_height_percent){
    float absolute = ((desired_height_percent/100)*0.2)+0.6;
    // impl_->locomotion_client_->SetStandHeight(absolute);
    impl_->send_lower_body_control_message("SetStandHeight","DriverUnitreeH1::set_stand_height_percent",{absolute});
}

void DriverUnitreeH1::enter_dummy_mode(){
    std::cout<<"DRIVER ENTERING DUMMY MODE! This will disable some safety checks! Do not do this if you are connected to a real robot!"<<std::endl;
    impl_->is_dummy_robot_ = true;
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
        throw std::runtime_error("[DriverUnitreeH1::change_control_mode] Function called when robot is not properly initialised!");
    }

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
}

/**
 * @brief Sends position commands to all upper body joints.
 * @param desired_joint_positions_rad A VectorXd of desired joint positions in radians.
 * @throws runtime_error if the robot is not initialized, if the robot is not in position control mode, or if the input size is incorrect.
 */
void DriverUnitreeH1::set_upper_body_joint_positions(const VectorXd& desired_joint_positions_rad) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_positions] Function called when robot is not properly initialised!");
    }

    if(current_mode_!=MODE::POSITION_CONTROLLED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_positions] Function called when robot is not in position control mode!");
    }

    if(desired_joint_positions_rad.size()!=upper_body_joints_.size()){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_positions] Input has incorrect size! (got "+std::to_string(desired_joint_positions_rad.size())+", expected "+std::to_string(upper_body_joints_.size())+")");
    }

    set_all_upper_body_joint_position_commands_(desired_joint_positions_rad);

    // Set weight to 1.0, to ensure that control instruction is followed
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);

    // Send message
    impl_->send_upper_body_control_message("DriverUnitreeH1::set_upper_body_joint_positions");
}

/**
 * @brief Sends velocity commands to all upper body joints.
 * @param desired_joint_velocities_rad_per_sec A VectorXd of desired joint velocities in radians per second.
 * @throws runtime_error if the robot is not initialized, if the robot is not in velocity control mode, or if the input size is incorrect.
 */
void DriverUnitreeH1::set_upper_body_joint_velocities(const VectorXd& desired_joint_velocities_rad_per_sec) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] Function called when robot is not properly initialised!");
    }

    if(current_mode_!=MODE::VELOCITY_CONTROLLED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] Function called when robot is not in velocity control mode!");
    }

    if(desired_joint_velocities_rad_per_sec.size()!=upper_body_joints_.size()){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] Input has incorrect size! (got "+std::to_string(desired_joint_velocities_rad_per_sec.size())+", expected "+std::to_string(upper_body_joints_.size())+")");
    }

    set_all_upper_body_joint_velocity_commands_(desired_joint_velocities_rad_per_sec);

    // Set weight to 1.0, to ensure that control instruction is followed
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);

    // Send message
    impl_->send_upper_body_control_message("set_upper_body_joint_velocities");
}

/**
 * @brief Sends torque commands to all upper body joints.
 * @param desired_joint_torques_Nm A VectorXd of desired joint torques in Newton-meters.
 * @throws runtime_error if the robot is not initialized, if the robot is not in torque control mode, or if the input size is incorrect.
 */
void DriverUnitreeH1::set_upper_body_joint_torques(const VectorXd& desired_joint_torques_Nm) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] Function called when robot is not properly initialised!");
    }

    if(current_mode_!=MODE::TORQUE_CONTROLLED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] Function called when robot is not in torque control mode!");
    }

    if(desired_joint_torques_Nm.size()!=upper_body_joints_.size()){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] Input has incorrect size! (got "+std::to_string(desired_joint_torques_Nm.size())+", expected "+std::to_string(upper_body_joints_.size())+")");
    }

    set_all_upper_body_joint_torque_commands_(desired_joint_torques_Nm);

    // Set weight to 1.0, to ensure that control instruction is followed
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);

    // Send message
    impl_->send_upper_body_control_message("DriverUnitreeH1::set_upper_body_joint_torques");
}

/**
 * @brief Sends torso velocity commands to the locomotion client.
 * @param desired_torso_velocity_mps_radps A VectorXd of size 3 containing desired {vx, vy, omega}.
 * @throws runtime_error if the robot is not initialized or if the input size is not 3.
 */
void DriverUnitreeH1::set_torso_velocity(const VectorXd& desired_torso_velocity_mps_radps) {

    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::set_torso_velocity] Function called when robot is not properly initialised!");
    }

    if(desired_torso_velocity_mps_radps.size()!=3){
        throw std::runtime_error("[DriverUnitreeH1::set_torso_velocity] Input has incorrect size! (got "+std::to_string(desired_torso_velocity_mps_radps.size())+", expected 3 -> {x, y, omega})");
    }

    float vx, vy, v_yaw;
    vx = desired_torso_velocity_mps_radps(0);
    vy = desired_torso_velocity_mps_radps(1);
    v_yaw = desired_torso_velocity_mps_radps(2);
    // impl_->locomotion_client_->SetVelocity(vx, vy, v_yaw, impl_->comms_timeout_sec_);
    impl_->send_lower_body_control_message("SetVelocity","DriverUnitreeH1::set_torso_velocity",{vx, vy, v_yaw, impl_->comms_timeout_sec_});
}

// #############################################
//  DriverUnitreeH1 private member functions
// #############################################

/**
 * @brief Writes position commands to all upper body joints without sending the message.
 * @param target_positions_rad A VectorXd of target joint positions in radians.
 */
void DriverUnitreeH1::set_all_upper_body_joint_position_commands_(const VectorXd& target_positions_rad){
    for(int i=0; i<upper_body_joints_.size(); i++){
        impl_->set_joint_position_command(upper_body_joints_.at(i),target_positions_rad(i));
        
    }
}

/**
 * @brief Writes velocity commands to all upper body joints without sending the message.
 * @param target_velocities_rad_per_sec A VectorXd of target joint velocities in radians per second.
 */
void DriverUnitreeH1::set_all_upper_body_joint_velocity_commands_(const VectorXd& target_velocities_rad_per_sec){
    for(int i=0; i<upper_body_joints_.size(); i++){
        impl_->set_joint_velocity_command(upper_body_joints_.at(i),target_velocities_rad_per_sec(i));
    }
}

/**
 * @brief Writes torque commands to all upper body joints without sending the message.
 * @param target_torques_Nm A VectorXd of target joint torques in Newton-meters.
 */
void DriverUnitreeH1::set_all_upper_body_joint_torque_commands_(const VectorXd& target_torques_Nm){
    for(int i=0; i<upper_body_joints_.size(); i++){
        impl_->set_joint_torque_command(upper_body_joints_.at(i),target_torques_Nm(i));
    }
}

/**
 * @brief Configures all upper body joints to damping mode without sending the command.
 */
void DriverUnitreeH1::damp_all_upper_body_joints_(){
    for(int i=0; i<upper_body_joints_.size(); i++){
        impl_->set_joint_damping_mode_command(upper_body_joints_.at(i));
    }
}

/**
 * @brief Safely starts upper body joints by entering damping mode and then ramping the position gain.
 *        This prepares the upper body joints for normal commanded motion.
 */
void DriverUnitreeH1::safely_start_upper_body_joints_(){
    float num_time_steps = static_cast<float>(impl_->weight_ramp_overall_duration_sec_/impl_->expected_firmware_update_period_chrono_sec_);

    // Start by putting the robot into "upper body damping mode"
    damp_all_upper_body_joints_();
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);
    impl_->send_upper_body_control_message("DriverUnitreeH1::safely_start_upper_body_joints_");
    std::this_thread::sleep_for(impl_->weight_ramp_overall_duration_sec_);

    // Then gradually ramp up the position gain to get the motors into the right place.
    for(int i=0; i<num_time_steps; i++){
        for(int j=0; j<upper_body_joints_.size(); j++){
            auto &cmd = impl_->cmd_msg_.motor_cmd().at(upper_body_joints_.at(j));
            cmd.q(0);
            cmd.dq(0.0);
            cmd.kp(0.0 + (static_cast<float>(i) / num_time_steps)*(impl_->kp_-0));
            cmd.kd(impl_->pos_cmd_kd_);
            cmd.tau(0);
        }
        impl_->send_upper_body_control_message("DriverUnitreeH1::safely_start_upper_body_joints_");
        std::this_thread::sleep_for(impl_->expected_firmware_update_period_chrono_sec_);
    }
}

/**
 * @brief Safely stops upper body joints by switching them to damping mode.
 */
void DriverUnitreeH1::safely_stop_upper_body_joints_(){
    float num_time_steps = static_cast<float>(impl_->weight_ramp_overall_duration_sec_/impl_->expected_firmware_update_period_chrono_sec_);

    // Start by putting the robot back into "upper body damping mode"
    damp_all_upper_body_joints_();
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);
    impl_->send_upper_body_control_message("DriverUnitreeH1::safely_stop_upper_body_joints_");
    std::this_thread::sleep_for(impl_->weight_ramp_overall_duration_sec_);
}