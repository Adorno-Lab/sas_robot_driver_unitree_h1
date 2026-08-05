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

/** To Do List:
 * Add briefs for all functions
 * Add copyright statement
 * Add tracking for upper body control mode (position, velocity, or torque)
 * Add input length validation to vector-valued setter functions
 * Add joint limit tracking and enforcement
 * Add joint velocity limit enforcement
 * Add checks to setter functions to make sure we're in the right mode (position / velocity / torque)
 */

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
    static constexpr std::chrono::duration<double> weight_ramp_step_time_sec_ {0.02};

    // #############################################
    //  Impl member functions
    // #############################################

    Impl() = default;

    /**
     * @brief DriverUnitreeB1::Impl::set_joint_position_command helper function for position commands. 
     *        Called in a few places when joint positions are being written to the motors. Centralises 
     *        the logic for this so it can be changed in only one place.
     * @param joint_id The id of the joint to be written to in Unitree SDK terms.
     * @param target_position_rad The joint position in radians
     */
    void set_joint_position_command(int joint_id, float target_position_rad)
    {
        auto &cmd = cmd_msg_.motor_cmd().at(joint_id);

        // Motors are torque controlled using the eqn:
        // Torque = kp * (q_des - q_curr) + kd * (dq_des - dq_curr) + tau_ff
        // So, here we set dq_des as 0, so there is some damping proportional to the speed.
        cmd.q(target_position_rad);
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
        cmd.dq(target_velocity_rad_per_sec);
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
     * @brief DriverUnitreeB1::Impl::send_upper_body_control_message helper function for the publisher. 
     *        Sends a message to the upper_body_publisher_ and throws a runtime_error if the write operation fails.
     *        Note that this function succeeding is not evidence that the robot received the message, only that the
     *        message was written to the publisher. There may be other problems that prevent it from receiving that 
     *        message (e.g., if the robot doesn't subscribe to the topic, or isn't powered on). A failure of this
     *        function usually indicates that something is wrong with the upper_body_publisher_ itself.
     * @param calling_function The function calling send_upper_body_control_message, used for informative runtime errors.
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
     */
    void check_upper_body_subscriber_setup(std::string calling_function){

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
     *        Makes function calls to the locomotion client and checks the return values. If any error codes are returned
     *        then we throw an exception.
     * @param message The member function of Impl::locomotion_client_ to be called.
     * @param calling_function The function calling send_lower_body_control_message, used for informative runtime errors.
     * @param args Any arguments required for the function call.
     */
    float send_lower_body_control_message(std::string message, std::string calling_function, std::vector<float> args){
        int32_t loco_client_return_code = 0;
        float this_function_return_value = 0.0f;

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

    void check_robot_still_connected(){  
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

DriverUnitreeH1::DriverUnitreeH1(std::string network_interface){

    // Create implementation object
    impl_ = std::make_shared<DriverUnitreeH1::Impl>();

    // Process arguments
    impl_->network_interface_ = network_interface;

    // Enact defaults
    current_mode_ = MODE::POSITION_CONTROLLED;
    
}

void DriverUnitreeH1::connect(){

    std::cout << "Connecting..." << std::endl;

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

void DriverUnitreeH1::initialize(){
    std::cout << "Initialising..." << std::endl;
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

void DriverUnitreeH1::deinitialize(){
    // IMPORTANT NOTE: This function is called by the SAS destructor. Therfore, for safety reasons, it must not
    // generate any exceptions.

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

void DriverUnitreeH1::disconnect(){
    // IMPORTANT NOTE: This function is called by the SAS destructor. Therfore, for safety reasons, it must not
    // generate any exceptions.

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

VectorXd DriverUnitreeH1::get_upper_body_joint_temperatures() const {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_temperatures] Function called when robot is not properly initialised!");
    }

    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    VectorXd current_j_casing_temp_C = VectorXd::Zero(upper_body_joints_.size());
    
    for (int i = 0; i < upper_body_joints_.size(); ++i) {
        current_j_casing_temp_C(i) = static_cast<double>(impl_->state_msg_.motor_state().at(upper_body_joints_.at(i)).temperature());
    }
    return current_j_casing_temp_C;
}

// --------------------------------------------
//  IMU getter functions
// --------------------------------------------

VectorXd DriverUnitreeH1::get_torso_velocity() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_torso_velocity] Function called when robot is not properly initialised!");
    }
    
    // Check that the subscriber is still working
    impl_->check_robot_still_connected();

    std::cout<<"DriverUnitreeH1::get_torso_velocity is not yet implemented"<<std::endl;
    return VectorXd::Zero(3);
}

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
    return imu_quat;
}

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

float DriverUnitreeH1::get_stand_height_percent() const {
    float stand_height;
    // impl_->locomotion_client_->GetStandHeight(stand_height);
    stand_height = impl_->send_lower_body_control_message("GetStandHeight","DriverUnitreeH1::get_stand_height_percent",{});
    float percent = (stand_height-0.6)/(0.2)*100;
    return(percent);
}

void DriverUnitreeH1::set_stand_height_percent(const float desired_height_percent){
    float absolute = ((desired_height_percent/100)*0.2)+0.6;
    // impl_->locomotion_client_->SetStandHeight(absolute);
    impl_->send_lower_body_control_message("SetStandHeight","DriverUnitreeH1::set_stand_height_percent",{absolute});
}

// --------------------------------------------
//  Setter functions
// --------------------------------------------

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

void DriverUnitreeH1::set_upper_body_joint_positions(const VectorXd& desired_joint_positions_rad) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_positions] Function called when robot is not properly initialised!");
    }

    if(current_mode_!=MODE::POSITION_CONTROLLED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_positions] Function called when robot is not in position control mode!");
    }

    set_all_upper_body_joint_position_commands_(desired_joint_positions_rad);

    // Set weight to 1.0, to ensure that control instruction is followed
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);

    // Send message
    impl_->send_upper_body_control_message("DriverUnitreeH1::set_upper_body_joint_positions");
}

void DriverUnitreeH1::set_upper_body_joint_velocities(const VectorXd& desired_joint_velocities_rad_per_sec) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] Function called when robot is not properly initialised!");
    }

    if(current_mode_!=MODE::VELOCITY_CONTROLLED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] Function called when robot is not in velocity control mode!");
    }

    set_all_upper_body_joint_velocity_commands_(desired_joint_velocities_rad_per_sec);

    // Set weight to 1.0, to ensure that control instruction is followed
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);

    // Send message
    impl_->send_upper_body_control_message("set_upper_body_joint_velocities");
}

void DriverUnitreeH1::set_upper_body_joint_torques(const VectorXd& desired_joint_velocities_rad_per_sec) {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] Function called when robot is not properly initialised!");
    }

    if(current_mode_!=MODE::TORQUE_CONTROLLED){
        throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] Function called when robot is not in torque control mode!");
    }

    set_all_upper_body_joint_torque_commands_(desired_joint_velocities_rad_per_sec);

    // Set weight to 1.0, to ensure that control instruction is followed
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);

    // Send message
    impl_->send_upper_body_control_message("DriverUnitreeH1::set_upper_body_joint_torques");
}

void DriverUnitreeH1::set_torso_velocity(const VectorXd& desired_torso_velocity_mps_radps) {

    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::set_torso_velocity] Function called when robot is not properly initialised!");
    }

    // TODO: REFUSE IF not in high-level mode, or not in velocity mode
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
 * @brief DriverUnitreeB1::set_all_upper_body_joint_position_commands_ private helper function for position commands. 
 *        Used to write position commands to all upper body joints at once
 * @param target_position_rad The joint positions in radians
 */
void DriverUnitreeH1::set_all_upper_body_joint_position_commands_(const VectorXd& target_positions_rad){
    for(int i=0; i<upper_body_joints_.size(); i++){
        impl_->set_joint_position_command(upper_body_joints_.at(i),target_positions_rad(i));
        
    }
}

/**
 * @brief DriverUnitreeB1::set_all_upper_body_joint_velocity_commands_ private helper function for velocity commands. 
 *        Used to write velocity commands to all upper body joints at once
 * @param target_position_rad The joint velocities in radians
 */
void DriverUnitreeH1::set_all_upper_body_joint_velocity_commands_(const VectorXd& target_velocities_rad_per_sec){
    for(int i=0; i<upper_body_joints_.size(); i++){
        impl_->set_joint_velocity_command(upper_body_joints_.at(i),target_velocities_rad_per_sec(i));
    }
}

/**
 * @brief DriverUnitreeB1::set_all_upper_body_joint_torque_commands_ private helper function for torque commands. 
 *        Used to write torque commands to all upper body joints at once
 * @param target_torques_Nm The joint torques in Newton metres
 */
void DriverUnitreeH1::set_all_upper_body_joint_torque_commands_(const VectorXd& target_torques_Nm){
    for(int i=0; i<upper_body_joints_.size(); i++){
        impl_->set_joint_torque_command(upper_body_joints_.at(i),target_torques_Nm(i));
    }
}

void DriverUnitreeH1::damp_all_upper_body_joints_(){
    for(int i=0; i<upper_body_joints_.size(); i++){
        impl_->set_joint_damping_mode_command(upper_body_joints_.at(i));
    }
}

void DriverUnitreeH1::safely_start_upper_body_joints_(){
    float num_time_steps = static_cast<float>(impl_->weight_ramp_overall_duration_sec_/impl_->weight_ramp_step_time_sec_);

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
        std::this_thread::sleep_for(impl_->weight_ramp_step_time_sec_);
    }
}

void DriverUnitreeH1::safely_stop_upper_body_joints_(){
    float num_time_steps = static_cast<float>(impl_->weight_ramp_overall_duration_sec_/impl_->weight_ramp_step_time_sec_);

    // Start by putting the robot back into "upper body damping mode"
    damp_all_upper_body_joints_();
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);
    impl_->send_upper_body_control_message("DriverUnitreeH1::safely_stop_upper_body_joints_");
    std::this_thread::sleep_for(impl_->weight_ramp_overall_duration_sec_);
}