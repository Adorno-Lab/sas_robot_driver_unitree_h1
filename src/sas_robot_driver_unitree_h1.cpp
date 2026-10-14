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
#   Acknowledgement: This file was originally adapted from the file 
#                    src/sas_robot_driver_unitree_b1.cpp written by Juan Jose 
#                    Quiroz Omana (juanjose.quirozomana@manchester.ac.uk) for the 
#                    unitree B1 driver package:
#                    https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1/tree/main
#
# ################################################################
*/

#include <sas_robot_driver_unitree_h1/sas_robot_driver_unitree_h1.hpp>

#include "DriverUnitreeH1.hpp"
#include <iostream>
#include <memory>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>

#include <unitree_drivers/DriverUnitreeLowState.h>

namespace sas
{

namespace
{

using LIMB = DriverUnitreeLowState::LIMB;

/**
 * @brief One limb of the H1 (19 DoF).
 */
struct H1Limb
{
    std::string name;
    LIMB limb;
    std::vector<std::string> joint_names;
    std::vector<double> lower_limits;
    std::vector<double> upper_limits;
};

// Joint position limits taken from https://support.unitree.com/home/en/H1_developer/About_H1
const std::vector<H1Limb> kH1Limbs = {
    {"left_leg", LIMB::LEFT_LEG,
     {"LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnkle"},
     {-3.14, -0.43, -0.43, -0.26, -0.87},
     { 3.14,  0.43,  0.43,  2.05,  0.52}},
    {"right_leg", LIMB::RIGHT_LEG,
     {"RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnkle"},
     {-3.14, -0.43, -0.43, -0.26, -0.87},
     { 3.14,  0.43,  0.43,  2.05,  0.52}},
    {"waist", LIMB::WAIST,
     {"WaistYaw"},
     {-2.35},
     { 2.35}},
    {"left_arm", LIMB::LEFT_ARM,
     {"LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow"},
     {-2.87, -0.34, -1.30, -1.25},
     { 2.87,  3.11,  4.45,  2.61}},
    {"right_arm", LIMB::RIGHT_ARM,
     {"RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow"},
     {-2.87, -3.11, -4.45, -1.25},
     { 2.87,  0.34,  1.30,  2.61}},
};

VectorXd _to_vectorxd(const std::vector<double>& values)
{
    return Eigen::Map<const VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
}

}

/**
 * @brief Internal implementation details for the ROS 2 Unitree H1 driver wrapper.
 *
 * This private helper stores the underlying backend driver instance and the
 * runtime dummy-mode state used by the higher-level ROS 2 interface.
 */
class RobotDriverUnitreeH1::Impl
{
public:
   std::shared_ptr<DriverUnitreeH1> hardware_;
   bool is_dummy_;
   std::vector<LimbEntry> limbs_;

   Impl() = default;
};

// ###########################################
// Constructor and Lifecycle Functions
// ###########################################

RobotDriverUnitreeH1::RobotDriverUnitreeH1(std::shared_ptr<Node> &node,
                                           const RobotDriverUnitreeH1Configuration &configuration,
                                           const std::shared_ptr<ShutdownSignaler> &shutdown_signaler)
    :LeggedRobotDriver{shutdown_signaler},
    topic_prefix_{configuration.robot_name},
    configuration_{configuration},
    node_{node},
    timer_period_{0.002},
    print_count_{0},
    clock_{0.002}
{
   impl_ = std::make_unique<RobotDriverUnitreeH1::Impl>();

   impl_->hardware_ = std::make_shared<DriverUnitreeH1>(configuration_.network_interface, 
                                                               configuration_.mode, 
                                                               configuration_.ENTER_DAMPING_MODE_ON_DEINIT,
                                                               configuration_.DUMMY_MODE,
                                                               shutdown_signaler);

   impl_->is_dummy_ = configuration_.DUMMY_MODE;

   // Set up the limbs vector
   for (const auto& h1_limb : kH1Limbs)
   {
      auto driver = std::make_shared<RobotDriverUnitreeH1Limb>(impl_->hardware_,
                                                               h1_limb.limb,
                                                               h1_limb.name,
                                                               std::make_tuple(_to_vectorxd(h1_limb.lower_limits),
                                                                              _to_vectorxd(h1_limb.upper_limits)),
                                                               shutdown_signaler);
      impl_->limbs_.push_back({h1_limb.name, driver, h1_limb.joint_names});
   }

   // Create publishers and subscribers that aren't part of the base class
   publisher_IMU_state_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_prefix_ + "/get/IMU_state", 1);
   publisher_IMU_orientation_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix_ + "/get/imu_orientation",1);
   publisher_temperatures_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix_ + "/get/joint_temperatures",1);
   publisher_stand_height_ = node_->create_publisher<std_msgs::msg::Float64>(topic_prefix_ + "/get/stand_height",1);

   subscriber_target_twist_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      topic_prefix_ + "/set/target_twist",
      1,
      std::bind(&RobotDriverUnitreeH1::_callback_target_twist, this, std::placeholders::_1)
   );

   subscriber_target_stand_height_ = node_->create_subscription<std_msgs::msg::Float64>(
      topic_prefix_ + "/set/stand_height",
      1,
      std::bind(&RobotDriverUnitreeH1::_callback_target_stand_height, this, std::placeholders::_1)
   );

   subscriber_set_control_mode_ = node_->create_subscription<std_msgs::msg::String>(
      topic_prefix_ + "/set/control_mode",
      1,
      std::bind(&RobotDriverUnitreeH1::_callback_set_control_mode, this, std::placeholders::_1)
   );

   // Register the control-loop callback that updates the ROS 2 state and applies any pending commands.
   set_control_loop_callback([this]() {
      try {
         _read_imu_state_and_publish();
      } catch (const std::exception& e) {
         std::cout << "[ERROR] [RobotDriverUnitreeH1::_read_imu_state_and_publish] Exception caught: "<<e.what()<<std::endl;
      }

      try {
         _read_temperatures_and_publish();
      } catch (const std::exception& e) {
         std::cout << "[ERROR] [RobotDriverUnitreeH1::_read_temperatures_and_publish] Exception caught: "<<e.what()<<std::endl;
      }

      try {
         _read_stand_height_and_publish();
      } catch (const std::exception& e) {
         std::cout << "[ERROR] [RobotDriverUnitreeH1::_read_stand_height_and_publish] Exception caught: "<<e.what()<<std::endl;
      }

      try {
         _set_torso_velocities_from_subscriber();
      } catch (const std::exception& e) {
         std::cout << "[ERROR] [RobotDriverUnitreeH1::_set_torso_velocities_from_subscriber] Exception caught: "<<e.what()<<std::endl;
      }

      try {
         _set_stand_height_from_subscriber();
      } catch (const std::exception& e) {
         std::cout << "[ERROR] [RobotDriverUnitreeH1::_set_stand_height_from_subscriber] Exception caught: "<<e.what()<<std::endl;
      }
    });
}

RobotDriverUnitreeH1::~RobotDriverUnitreeH1() = default;

/**
 * @brief Establish communications with the robot hardware. Calls `DriverUnitreeH1::connect()`.
 */
void RobotDriverUnitreeH1::connect()
{
   impl_->hardware_->connect();
}

/**
 * @brief Disconnects from the robot hardware, freeing communications resources. This function 
 * is called automatically during teardown. Calls `DriverUnitreeH1::disconnect()`.
 */
void RobotDriverUnitreeH1::disconnect()
{
   impl_->hardware_->disconnect();
}

/**
 * @brief Initialize the robot after the communication channels have been opened. This will start
 * the locomotion server, which will cause the robot to take steps autonomously if needed to
 * balance itself. 
 * 
 * After this function has been called, the robot will begin to respond to user commands.
 * 
 * Calls DriverUnitreeH1::initialize()
 */
void RobotDriverUnitreeH1::initialize()
{
   impl_->hardware_->initialize();
}

/**
 * @brief Safely deinitialize the robot and bring motion to a safe shutdown state. This function 
 * is called automatically during teardown. Calls DriverUnitreeH1::deinitialize()
 */
void RobotDriverUnitreeH1::deinitialize()
{
   impl_->hardware_->deinitialize();
}


// ###########################################
// State Getters
// ###########################################

/**
 * @brief Read the current upper-body joint velocities from the backend driver.
 * @return A VectorXd containing the joint velocities in radians per second.
 */
VectorXd RobotDriverUnitreeH1::get_joint_velocities()
{
   return impl_->hardware_->get_upper_body_joint_velocities();
}

/**
 * @brief Read the latest estimated upper-body joint torques.
 * @return A VectorXd containing the joint torques in newton-metres.
 */
VectorXd RobotDriverUnitreeH1::get_joint_torques()
{
   return impl_->hardware_->get_upper_body_joint_torques();
}

/**
 * @brief Read the latest imu orientation measurement.
 * @return A DQ containing the most recent orientation.
 */
DQ RobotDriverUnitreeH1::get_orientation()
{
   DriverUnitreeLowState::IMUData imu_data = impl_->hardware_->get_IMU_data();

   // If the data is invalid (i.e., it did not come from a real measurement) then stop the process
   // right here, return zero.
   if (!imu_data.valid || r.vec4().norm() == 0.0){
        return DQ(0);
   }

   DQ orientation = DQ(
      imu_data.quaternion.at(0), //w
      imu_data.quaternion.at(1), //x
      imu_data.quaternion.at(2), //y
      imu_data.quaternion.at(3)).normalize(); //z

   return orientation;
}

/**
 * @brief Read the latest angular velocity measurement from the gyroscope.
 * @return A DQ containing the most recent angular velocity measurement.
 */
DQ RobotDriverUnitreeH1::get_angular_velocity()
{
   DriverUnitreeLowState::IMUData imu_data = impl_->hardware_->get_IMU_data();

   // If the data is invalid (i.e., it did not come from a real measurement) then stop the process
   // right here, return zero.
   if (!imu_data.valid){
        return DQ(0);
   }

   DQ angular_velocity = DQ(
      0, //w
      imu_data.gyroscope.at(0), //x
      imu_data.gyroscope.at(1), //y
      imu_data.gyroscope.at(2)); //z

   return angular_velocity;

}

/**
 * @brief Read the latest linear acceleration measurement from the accelerometer.
 * @return A DQ containing the most recent linear acceleration measurement.
 */
DQ RobotDriverUnitreeH1::get_linear_acceleration()
{
   DriverUnitreeLowState::IMUData imu_data = impl_->hardware_->get_IMU_data();

   // If the data is invalid (i.e., it did not come from a real measurement) then stop the process
   // right here, return zero.
   if (!imu_data.valid){
        return DQ(0);
   }

   DQ linear_acceleration = DQ(
      0, //w
      imu_data.accelerometer.at(0), //x
      imu_data.accelerometer.at(1), //y
      imu_data.accelerometer.at(2)); //z

   return angular_velocity;

}

/**
 * @brief Get the high level modes supported by this driver.
 * @return A vector of supported modes.
 */
std::vector<LeggedRobotDriver::HIGH_LEVEL_MODE> RobotDriverUnitreeH1::get_supported_high_level_modes() const
{
    return {HIGH_LEVEL_MODE::IDLE, HIGH_LEVEL_MODE::WALKING};
}

/**
 * @brief Check if a given functionality is supported by this driver. Used by the base class to determine what
 * commands are accepted.
 * @return True if the functionality is supported, false if not.
 */
bool RobotDriverUnitreeH1::is_supported(const LEGGED_FUNCTIONALITY &functionality) const
{
   switch(functionality){
      case LEGGED_FUNCTIONALITY::TWIST:
         return true;
      case LEGGED_FUNCTIONALITY::BASE_HEIGHT:
         return true;
      case LEGGED_FUNCTIONALITY::BASE_ORIENTATION:
         return false;
      default:
         // If there are any other functionalities later introduced that I don't know about at time of 
         // writing, then return false for those.
         return false;
   }
}

/**
 * @brief Get a vector of limbs describing the robot's layout. Called by the base class to determine
 * what limbs the robot has.
 * @return A vector of limbs.
 */
std::vector<LeggedRobotDriver::LimbEntry> RobotDriverUnitreeH1::get_limbs() const
{
   return impl_->limbs_;
}


// ###########################################
// Target Setters
// ###########################################

/**
 * @brief Update the high level control mode. Driver always starts in IDLE mode.
 * @return A LeggedRobotDriver::HIGH_LEVEL_MODE (IDLE, STANDING or WALKING).
 */
void RobotDriverUnitreeH1::_set_high_level_mode(const HIGH_LEVEL_MODE &mode)
{
   // The H1s capabilities are unrelated to whether its walking or standing. In order to
   // maintain consistency with other legged robot drivers, we communicate this by saying
   // that we don't support a dedicated standing mode.

   // Stop moving for the mode change, because the other legged drivers do this too.
   impl_->hardware_->set_target_high_level_velocities({0.0, 0.0, 0.0});
   
   if(mode==HIGH_LEVEL_MODE::STANDING){
      throw std::runtime_error("[RobotDriverUnitreeH1::_set_high_level_mode] Unitree H1 does not support standing mode!"); 
   }

   // Then just return, because we don't actually need to do anything.
   return;
   
}

// /**
//  * @brief Forward upper-body joint position targets to the low-level H1 driver.
//  * @param desired_joint_positions_rad Desired joint positions in radians.
//  */
// void RobotDriverUnitreeH1::set_target_joint_positions(const VectorXd& desired_joint_positions_rad)
// {
//    impl_->hardware_->set_upper_body_joint_positions(desired_joint_positions_rad);
// }

// This function is required by the base class, although the target-twist command is handled in the control loop rather than through a dedicated subscriber. The periodic control-loop implementation remains the authoritative path for torso-velocity updates.
/**
 * @brief Set the target torso twist.
 * [DEPRECATED] Use of this function is no longer encouraged. Setting of target twists is 
 * now handled by the LeggedRobotDriver control loop callback function. 
 * @param twist Desired twist represented as a dual quaternion twist vector.
 */
void RobotDriverUnitreeH1::set_target_twist(const DQ& twist)
{
   const VectorXd twist_vec = twist.vec6();
    //    0  1  2  3  4  5
    //   wx wy wz vx vy vz
    double vx = twist_vec(3);
    double vy = twist_vec(4);
    double wz = twist_vec(2);
   VectorXd desired_velocity(3);
   desired_velocity << vx, vy, wz;

   impl_->hardware_->set_torso_velocity(desired_velocity);
}

/**
 * @brief Set the target base orientation.
 * This function is not supported because the high level Unitree API does
 * not contain functionality to control the robot's torso orientation directly.
 * @param r Target base orientation, unused.
 */
void RobotDriverUnitreeH1::set_target_base_orientation(const DQ &r)
{
   throw std::logic_error("[RobotDriverUnitreeH1::set_target_base_orientation]: This function is not supported.");
}

/**
 * @brief Set the robot stand height. Called by the LeggedRobotDriver control loop by default.
 * @param base_height Desired stand height in meters. Hardware clamps this to a safe range.
 */
void RobotDriverUnitreeH1::set_target_base_height(const double& base_height)
{
   impl_->hardware_->set_stand_height(base_height);
}

// ###########################################
// Additional Functions
// ###########################################

/**
 * @brief Publish the most recent IMU reading to the relevant ROS 2 topics.
 * Called by the LeggedRobotDriver control loop as an additional callback function. Calls DriverUnitreeH1::get_IMU_data().
 */
void RobotDriverUnitreeH1::_read_imu_state_and_publish()
{

   sensor_msgs::msg::Imu ros_msg_imu;
   ros_msg_imu.header.stamp = node_->get_clock()->now();

   geometry_msgs::msg::PoseStamped ros_msg_pose;
   ros_msg_pose.header.stamp = node_->get_clock()->now();

   DriverUnitreeLowState::IMUData imu_data = impl_->hardware_->get_IMU_data();

   // If the data is invalid (i.e., it did not come from a real measurement) then stop the process
   // right here, don't publish anything.
   if(!imu_data.valid){
      return;
   }

   DQ orientation = DQ(
      imu_data.quaternion.at(0), //w
      imu_data.quaternion.at(1), //x
      imu_data.quaternion.at(2), //y
      imu_data.quaternion.at(3)).normalize(); //z

   publisher_IMU_orientation_->publish(sas::dq_to_geometry_msgs_pose_stamped(orientation));

   VectorXd vec_orientation = orientation.vec4();
   ros_msg_imu.orientation.w = vec_orientation(0);
   ros_msg_imu.orientation.x = vec_orientation(1);
   ros_msg_imu.orientation.y = vec_orientation(2);
   ros_msg_imu.orientation.z = vec_orientation(3);

   ros_msg_imu.angular_velocity.x = imu_data.gyroscope.at(0);
   ros_msg_imu.angular_velocity.y = imu_data.gyroscope.at(1);
   ros_msg_imu.angular_velocity.z = imu_data.gyroscope.at(2);

   ros_msg_imu.linear_acceleration.x = imu_data.accelerometer.at(0);
   ros_msg_imu.linear_acceleration.y = imu_data.accelerometer.at(1);
   ros_msg_imu.linear_acceleration.z = imu_data.accelerometer.at(2);

   publisher_IMU_state_->publish(ros_msg_imu);
}

/**
 * @brief Publish the most recent joint temperature reading in degrees Celsius to the relevant ROS 2 topics.
 * Called by the LeggedRobotDriver control loop as an additional callback function. Calls DriverUnitreeH1::get_joint_temperatures().
 */
void RobotDriverUnitreeH1::_read_temperatures_and_publish()
{
   auto joint_temps = impl_->hardware_->get_joint_temperatures();
   
   std_msgs::msg::Float64MultiArray msg;

   msg.data.reserve(joint_temps.size());

   msg.data.insert(msg.data.end(),
                  joint_temps.data(),
                  joint_temps.data() + joint_temps.size());
   publisher_temperatures_->publish(msg);
}

/**
 * @brief Publish the current stand height in meters to the relevant ROS 2 topics.
 * Called by the LeggedRobotDriver control loop as an additional callback function. Calls DriverUnitreeH1::get_stand_height().
 */
void RobotDriverUnitreeH1::_read_stand_height_and_publish()
{
   auto stand_height = impl_->hardware_->get_stand_height();
      
   std_msgs::msg::Float64 msg;

   msg.data=stand_height;

   publisher_stand_height->publish(msg);
}

/**
 * @brief Store the most recent target twist from the ROS subscription.
 * @param msg Incoming twist message.
 */
void RobotDriverUnitreeH1::_callback_target_twist(const geometry_msgs::msg::TwistStamped& msg)
{
   target_twist_ <<msg.twist.angular.x,
                  msg.twist.angular.y,
                  msg.twist.angular.z,
                  msg.twist.linear.x,
                  msg.twist.linear.y,
                  msg.twist.linear.z;

   new_target_twist_available_ = true;
}

/**
 * @brief Store the most recent stand-height command from the ROS subscription.
 * @param msg Incoming stand-height command message.
 */
void RobotDriverUnitreeH1::_callback_target_stand_height_percent(const std_msgs::msg::Float64& msg)
{
   target_stand_height_percent_=msg.data;

   new_target_stand_height_percent_available_ = true;
}

/**
 * @brief Change the active control mode using a ROS 2 string message.
 * @param msg Incoming control-mode command.
 */
void RobotDriverUnitreeH1::_callback_set_control_mode(const std_msgs::msg::String& msg)
{
   impl_->hardware_->change_control_mode(msg.data);
}

/**
 * @brief Apply a pending torso-velocity command from the subscriber queue.
 */
void RobotDriverUnitreeH1::_set_torso_velocities_from_subscriber()
{
    if (new_target_twist_available_)
    {
        //    0  1  2  3  4  5
        //   wx wy wz vx vy vz
        VectorXd desired_velocities(3);
        desired_velocities << target_twist_(3), target_twist_(4), target_twist_(2);

        impl_->hardware_->set_torso_velocity(desired_velocities);

        new_target_twist_available_ = false;
    }
}

/**
 * @brief Apply a pending stand-height command from the subscriber queue.
 */
void RobotDriverUnitreeH1::_set_stand_height_percent_from_subscriber()
{
   if (new_target_stand_height_percent_available_)
   {
      impl_->hardware_->set_stand_height_percent(target_stand_height_percent_);
      new_target_stand_height_percent_available_ = false;
   }
}

/**
 * @brief Forward joint-velocity targets to the backend driver.
 * @param desired_joint_velocities_radps Target joint velocities in radians per second.
 */
void RobotDriverUnitreeH1::set_target_joint_velocities(const VectorXd& desired_joint_velocities_radps){
   impl_->hardware_->set_upper_body_joint_velocities(desired_joint_velocities_radps);
}

/**
 * @brief Forward joint-torque targets to the backend driver.
 * @param desired_joint_torques_Nm Target joint torques in newton-metres.
 */
void RobotDriverUnitreeH1::set_target_joint_torques(const VectorXd& desired_joint_torques_Nm){
   impl_->hardware_->set_upper_body_joint_torques(desired_joint_torques_Nm);
}

} // End of namespace sas