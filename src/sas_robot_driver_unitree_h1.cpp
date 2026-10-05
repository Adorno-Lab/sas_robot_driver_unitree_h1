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

#include <sas_robot_driver_unitree_h1/sas_robot_driver_unitree_h1.hpp>

#include "DriverUnitreeH1.hpp"
#include <iostream>
#include <memory>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>

namespace sas
{

/**
 * @brief Internal implementation class for the ROS 2 Unitree H1 driver wrapper.
 *
 * This object stores the underlying hardware driver instance and the runtime
 * dummy-mode state used by the higher-level ROS 2 interface.
 */
class RobotDriverUnitreeH1::Impl
{
public:
   std::shared_ptr<DriverUnitreeH1> unitree_h1_driver_;
   bool is_dummy_;

   Impl() = default;
};

// ###########################################
// Required Functions due to inheritance
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

    impl_->unitree_h1_driver_ = std::make_shared<DriverUnitreeH1>(configuration_.network_interface, 
                                                                  configuration_.mode, 
                                                                  configuration_.ENTER_DAMPING_MODE_ON_DEINIT,
                                                                  configuration_.DUMMY_MODE,
                                                                  shutdown_signaler);

   impl_->is_dummy_ = configuration_.DUMMY_MODE;

   // Create publishers and subscribers that aren't part of the base class
   publisher_IMU_state_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_prefix_ + "/get/IMU_state", 1);
   publisher_IMU_orientation_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix_ + "/get/imu_orientation",1);
   publisher_temperatures_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix_ + "/get/temperatures",1);
   publisher_stand_height_percent_ = node_->create_publisher<std_msgs::msg::Float64>(topic_prefix_ + "/get/stand_height_percent",1);

   subscriber_target_twist_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      topic_prefix_ + "/set/target_twist",
      1,
      std::bind(&RobotDriverUnitreeH1::_callback_target_twist, this, std::placeholders::_1)
   );

   subscriber_target_stand_height_percent_ = node_->create_subscription<std_msgs::msg::Float64>(
      topic_prefix_ + "/set/stand_height_percent",
      1,
      std::bind(&RobotDriverUnitreeH1::_callback_target_stand_height_percent, this, std::placeholders::_1)
   );

   subscriber_set_control_mode_ = node_->create_subscription<std_msgs::msg::String>(
      topic_prefix_ + "/set/control_mode",
      1,
      std::bind(&RobotDriverUnitreeH1::_callback_set_control_mode, this, std::placeholders::_1)
   );

   // Register the control-loop callback that updates the ROS 2 state and applies any pending commands.
   set_control_loop_callback([this]() {
      try {
         // _read_joint_states_and_publish();
         _read_imu_state_and_publish();
         _read_temperatures_and_publish();
         _read_stand_height_and_publish();
         // _read_battery_state();
         // _read_twist_state_and_publish();
         _set_torso_velocities_from_subscriber();
         _set_stand_height_percent_from_subscriber();
         
         // _set_target_velocities_from_subscriber();
         //_read_rpy_angles_state_and_publish();
      } catch (const std::exception& e) {
         std::cout << "[ERROR] [DriverUnitreeH1 Callback Function] Exception caught: "<<e.what()<<std::endl;
      }
    });
}

RobotDriverUnitreeH1::~RobotDriverUnitreeH1() = default;

/**
 * @brief Read the current upper-body joint positions from the backend driver.
 * @return Joint state vector in radians.
 */
VectorXd RobotDriverUnitreeH1::get_joint_positions()
{
   return impl_->unitree_h1_driver_->get_upper_body_joint_positions();
}

/**
 * @brief Forward joint-position targets to the low-level H1 driver.
 * @param desired_joint_positions_rad Target joint positions in radians.
 */
void RobotDriverUnitreeH1::set_target_joint_positions(const VectorXd& desired_joint_positions_rad)
{
   impl_->unitree_h1_driver_->set_upper_body_joint_positions(desired_joint_positions_rad);
}

/**
 * @brief Read the current upper-body joint velocities from the backend driver.
 * @return Joint velocity vector in radians per second.
 */
VectorXd RobotDriverUnitreeH1::get_joint_velocities()
{
   return impl_->unitree_h1_driver_->get_upper_body_joint_velocities();
}

/**
 * @brief Read the latest estimated upper-body joint torques.
 * @return Joint torque vector in newton-metres.
 */
VectorXd RobotDriverUnitreeH1::get_joint_torques()
{
   return impl_->unitree_h1_driver_->get_upper_body_joint_torques();
}

/**
 * @brief Connect the low-level Unitree backend and establish the ROS 2 control interface.
 */
void RobotDriverUnitreeH1::connect()
{
   impl_->unitree_h1_driver_->connect();
}

/**
 * @brief Disconnect the driver and release the underlying communication channels.
 */
void RobotDriverUnitreeH1::disconnect()
{
   impl_->unitree_h1_driver_->disconnect();
}

/**
 * @brief Initialize the robot after the communication channels have been opened.
 */
void RobotDriverUnitreeH1::initialize()
{
   impl_->unitree_h1_driver_->initialize();
}

/**
 * @brief Safely deinitialize the robot and bring motion to a safe shutdown state.
 */
void RobotDriverUnitreeH1::deinitialize()
{
   impl_->unitree_h1_driver_->deinitialize();
}

// This function is required by the base class, although the target-twist command is handled in the control loop rather than through a dedicated subscriber. The periodic control-loop implementation remains the authoritative path for torso-velocity updates.
/**
 * @brief Convert a target twist into the torso-velocity command expected by the backend.
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

   impl_->unitree_h1_driver_->set_torso_velocity(desired_velocity);
}

/**
 * @brief Set the target base orientation.
 *        This is currently unimplemented because the underlying Unitree H1 stack does not expose
 *        a direct equivalent API in the current driver integration.
 * @param r Target base orientation, currently unused.
 */
void RobotDriverUnitreeH1::set_target_base_orientation(const DQ& r)
{
   (void)r;
}

/**
 * @brief Set the robot stand height as a percentage of its configured operating range.
 * @param base_height Desired stand height percentage in the range supported by the backend.
 */
void RobotDriverUnitreeH1::set_target_base_height(const double& base_height)
{
   impl_->unitree_h1_driver_->set_stand_height_percent(base_height);
}

// ###########################################
// Additional Functions
// ###########################################

/**
 * @brief Publish the current IMU state and orientation information to ROS 2 topics.
 */
void RobotDriverUnitreeH1::_read_imu_state_and_publish()
{
   sensor_msgs::msg::Imu ros_msg_imu;
   ros_msg_imu.header.stamp = node_->get_clock()->now();

   geometry_msgs::msg::PoseStamped ros_msg_pose;
   ros_msg_pose.header.stamp = node_->get_clock()->now();

   DQ orientation = impl_->unitree_h1_driver_->get_IMU_orientation();

   if (is_unit(orientation)){
      VectorXd vec_orientation = orientation.vec4();
      ros_msg_imu.orientation.w = vec_orientation(0);
      ros_msg_imu.orientation.x = vec_orientation(1);
      ros_msg_imu.orientation.y = vec_orientation(2);
      ros_msg_imu.orientation.z = vec_orientation(3);

      publisher_IMU_orientation_->publish(sas::dq_to_geometry_msgs_pose_stamped(orientation));

      VectorXd vec_angular_velocity = impl_->unitree_h1_driver_->get_gyroscope_data();
      ros_msg_imu.angular_velocity.x = vec_angular_velocity(0);
      ros_msg_imu.angular_velocity.y = vec_angular_velocity(1);
      ros_msg_imu.angular_velocity.z = vec_angular_velocity(2);

      VectorXd vec_acceleration = impl_->unitree_h1_driver_->get_accelerometer_data();
      ros_msg_imu.linear_acceleration.x = vec_acceleration(0);
      ros_msg_imu.linear_acceleration.y = vec_acceleration(1);
      ros_msg_imu.linear_acceleration.z = vec_acceleration(2);

      publisher_IMU_state_->publish(ros_msg_imu);
   }
}

/**
 * @brief Publish the joint and IMU temperature readings to the configured ROS topic.
 */
void RobotDriverUnitreeH1::_read_temperatures_and_publish()
{
   auto joint_temps = impl_->unitree_h1_driver_->get_joint_temperatures();
   auto IMU_temp = impl_->unitree_h1_driver_->get_IMU_temperature();
   
   std_msgs::msg::Float64MultiArray msg;

   msg.data.reserve(joint_temps.size() + 1);

   msg.data.push_back(IMU_temp);
   msg.data.insert(msg.data.end(),
                  joint_temps.data(),
                  joint_temps.data() + joint_temps.size());
   publisher_temperatures_->publish(msg);
}

/**
 * @brief Publish the current stand-height percentage to a ROS 2 topic.
 */
void RobotDriverUnitreeH1::_read_stand_height_and_publish()
{
   auto stand_height_percent = impl_->unitree_h1_driver_->get_stand_height_percent();
      
   std_msgs::msg::Float64 msg;

   msg.data=stand_height_percent;

   publisher_stand_height_percent_->publish(msg);
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
   impl_->unitree_h1_driver_->change_control_mode(msg.data);
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

        impl_->unitree_h1_driver_->set_torso_velocity(desired_velocities);

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
      impl_->unitree_h1_driver_->set_stand_height_percent(target_stand_height_percent_);
      new_target_stand_height_percent_available_ = false;
   }
}

/**
 * @brief Forward joint-velocity targets to the backend driver.
 * @param desired_joint_velocities_radps Target joint velocities in radians per second.
 */
void RobotDriverUnitreeH1::set_target_joint_velocities(const VectorXd& desired_joint_velocities_radps){
   impl_->unitree_h1_driver_->set_upper_body_joint_velocities(desired_joint_velocities_radps);
}

/**
 * @brief Forward joint-torque targets to the backend driver.
 * @param desired_joint_torques_Nm Target joint torques in newton-metres.
 */
void RobotDriverUnitreeH1::set_target_joint_torques(const VectorXd& desired_joint_torques_Nm){
   impl_->unitree_h1_driver_->set_upper_body_joint_torques(desired_joint_torques_Nm);
}

} // End of namespace sas