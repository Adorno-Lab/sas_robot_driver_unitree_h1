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

using namespace DQ_robotics;
using namespace Eigen;

/** To Do List:
 * Add briefs for all functions
 * Add copyright statement
 * See if there's a way to verify that the connection has been made successfully in DriverUnitreeH1::connect
 * Add tracking for the lower body level (low or high) and the control mode (position or velocity)
 * Decide in DriverUnitreeH1::initialize whether to start the robot locomotion driver based on the lower body level setting
 * Add input length validation to vector-valued setter functions
 * Add joint limit tracking and enforcement
 * Add joint velocity limit enforcement
 * Add checks to setter functions to make sure we're in the right mode (position / velocity) and level (low / high)
 * Add checking to make sure connections succeed, such as in connect and initialise, and that coms succeed, like in the motor writes.
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
    float current_control_weight_=0.0;

    static constexpr float comms_timeout_sec_ = 1.0f;
    static constexpr float kp_ = 60.f;
    static constexpr float pos_cmd_kd_ = 1.5f;
    static constexpr float vel_cmd_kd_ = 10.f;
    static constexpr std::chrono::duration<double> weight_ramp_overall_duration_sec_{2.0};
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
     * @brief DriverUnitreeB1::Impl::ramp_upper_body_weight helper function for init / deinit. 
     *        Ramps the control weight over Impl::weight_ramp_overall_duration_sec_ seconds
     * @param target_weight The desired upper body control weight (1.0 for init, 0.0 for deinit)
     */
    void ramp_upper_body_control_weight(float target_weight){
        
        // Send the message, gradually ramp the weight each iteration
        float num_time_steps = static_cast<float>(weight_ramp_overall_duration_sec_/weight_ramp_step_time_sec_);
        float starting_weight = current_control_weight_;
        float weight = starting_weight;
        for(int i=0; i<num_time_steps; i++){
            weight = starting_weight + (static_cast<float>(i) / num_time_steps)*(target_weight-starting_weight);
            cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(weight);
            upper_body_publisher_->Write(cmd_msg_);
            std::this_thread::sleep_for(weight_ramp_step_time_sec_);
            current_control_weight_ = weight;
        }

        // Make sure the weight is now the final one
        cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(target_weight);
        upper_body_publisher_->Write(cmd_msg_);
        current_control_weight_ = target_weight;
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
        auto &state = state_msg_.motor_state().at(joint_id);
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
        auto &state = state_msg_.motor_state().at(joint_id);
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
};

// #############################################
//  DriverUnitreeH1 public member functions
// #############################################

DriverUnitreeH1::DriverUnitreeH1(std::string network_interface){

    // Create implementation object
    impl_ = std::make_shared<DriverUnitreeH1::Impl>();

    // Process arguments
    impl_->network_interface_ = network_interface;
    
}

void DriverUnitreeH1::connect(){
    // Initialize the robot communication system with the given network interface.
    unitree::robot::ChannelFactory::Instance()->Init(0, impl_->network_interface_);

    // Start high level locomotion client.
    impl_->locomotion_client_ = std::make_shared<unitree::robot::h1::LocoClient>();
    impl_->locomotion_client_->Init(); // initialize the connection
    impl_->locomotion_client_->SetTimeout(impl_->comms_timeout_sec_); // set a 10 second timeout

    // Start low level publisher
    impl_->upper_body_publisher_.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>("rt/arm_sdk"));
    impl_->upper_body_publisher_->InitChannel();

    // Start low level subscriber
    impl_->upper_body_subscriber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lf/lowstate"));
    impl_->upper_body_subscriber_->InitChannel([&](const void *msg) {
        auto s = ( const unitree_go::msg::dds_::LowState_* )msg;
        memcpy( &impl_->state_msg_, s, sizeof( unitree_go::msg::dds_::LowState_ ) );
        }, 1);

    current_status_ = STATUS::CONNECTED;
}

void DriverUnitreeH1::initialize(){

    if(current_status_!=STATUS::CONNECTED){
        throw std::runtime_error("[DriverUnitreeH1::initialize] Initialize called when robot is not properly connected!");
    }

    std::cout << "Starting locomotion server..." << std::endl;
    impl_->locomotion_client_->Start();
    std::cout << "    Done." << std::endl;

    std::cout << "Initialising upper body joints..." << std::endl;
    // Send all upper body joints to their zero positions
    set_all_upper_body_joint_position_commands_(VectorXd::Zero(upper_body_joints_.size()));

    // Ramp up the control weight
    impl_->ramp_upper_body_control_weight(1.0);
    std::cout << "    Done." << std::endl;

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

    try{
        // Stop any ongoing motion
        std::cout << "Stopping locomotion server..." << std::endl;
        impl_->locomotion_client_->StopMove();
        std::cout << "    Done."<<std::endl;
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] [DriverUnitreeH1::deinitialize] Exception caught while stopping locomotion server: "<<e.what()<<std::endl;
    }

    // Put the robot into damping mode if requested
    if(impl_->enter_damping_mode_on_deinit_){
        try{
            std::cout << "Entering damping mode..." << std::endl;
            impl_->locomotion_client_->Damp();
            std::cout << "    Done." << std::endl;
        }
        catch (const std::exception& e){
            std::cout << "[ERROR] [DriverUnitreeH1::deinitialize] Exception caught while entering damping mode: "<<e.what()<<std::endl;
        }
    }
    else{
        try{
            std::cout << "Deinitialising upper body joints..." << std::endl;
            // Tell the joints to hold their current positions
            VectorXd current_positions = get_upper_body_joint_positions();
            set_all_upper_body_joint_position_commands_(current_positions);

            // Ramp down the control weight
            impl_->ramp_upper_body_control_weight(0.0);
            std::cout << "    Done." << std::endl;
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

    // Close the channels
    try{
        impl_->upper_body_publisher_->CloseChannel();
        impl_->upper_body_subscriber_->CloseChannel();
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] [DriverUnitreeH1::disconnect] Exception caught while closing upper body coms channels: "<<e.what()<<std::endl;
    }

    current_status_ = STATUS::DISCONNECTED;
}

// --------------------------------------------
//  Upper body getter functions
// --------------------------------------------

VectorXd DriverUnitreeH1::get_upper_body_joint_positions() const {
    
    if(current_status_!=STATUS::INITIALIZED){
        throw std::runtime_error("[DriverUnitreeH1::get_upper_body_joint_positions] Function called when robot is not properly initialised!");
    }

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
    
    std::cout<<"DriverUnitreeH1::get_torso_velocity is not yet implemented"<<std::endl;
    return VectorXd::Zero(3);
}

DQ DriverUnitreeH1::get_IMU_orientation() const {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::get_IMU_orientation] Function called when robot is not properly initialised!");
    }

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
//  Setter functions
// --------------------------------------------

void DriverUnitreeH1::set_upper_body_joint_positions(const VectorXd& desired_joint_positions_rad) {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_positions] Function called when robot is not properly initialised!");
    }

    set_all_upper_body_joint_position_commands_(desired_joint_positions_rad);

    // Set weight to 1.0, to ensure that control instruction is followed
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);

    // Send message
    impl_->upper_body_publisher_->Write(impl_->cmd_msg_);
}

void DriverUnitreeH1::set_upper_body_joint_velocities(const VectorXd& desired_joint_velocities_rad_per_sec) {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_velocities] Function called when robot is not properly initialised!");
    }

    set_all_upper_body_joint_velocity_commands_(desired_joint_velocities_rad_per_sec);

    // Set weight to 1.0, to ensure that control instruction is followed
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);

    // Send message
    impl_->upper_body_publisher_->Write(impl_->cmd_msg_);
}

void DriverUnitreeH1::set_upper_body_joint_torques(const VectorXd& desired_joint_velocities_rad_per_sec) {
    
    if(current_status_!=STATUS::INITIALIZED){
            throw std::runtime_error("[DriverUnitreeH1::set_upper_body_joint_torques] Function called when robot is not properly initialised!");
    }

    set_all_upper_body_joint_torque_commands_(desired_joint_velocities_rad_per_sec);

    // Set weight to 1.0, to ensure that control instruction is followed
    impl_->cmd_msg_.motor_cmd().at(JOINT_INDEX::kNotUsedJoint).q(1.0);

    // Send message
    impl_->upper_body_publisher_->Write(impl_->cmd_msg_);
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
    impl_->locomotion_client_->SetVelocity(vx, vy, v_yaw, impl_->comms_timeout_sec_);
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