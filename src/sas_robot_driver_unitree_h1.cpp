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


/* 
To Add:
- Set joint velocities and torques
- Get and set stand height
- IMU publishers
- Temperature publishers
- Subscriber for desired twist
*/

namespace sas
{

class RobotDriverUnitreeH1::Impl
{
public:
   std::shared_ptr<DriverUnitreeH1> unitree_h1_driver_;

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
                                                                  configuration_.ENTER_DAMPING_MODE_ON_DEINIT);

   if(configuration_.robot_name=="Dummy"){
      // This is the dummy robot, tell the driver that there is no real robot connected
      impl_->unitree_h1_driver_->enter_dummy_mode();
   }

   // Create publishers and subscribers that aren't part of the base class
   publisher_IMU_state_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_prefix_ + "/get/IMU_state", 1);
   publisher_IMU_orientation_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix_ + "/get/imu_orientation",1);

   // set the callback here
   set_control_loop_callback([this]() {
      try {
         // _read_joint_states_and_publish();
         _read_imu_state_and_publish();
         // _read_battery_state();
         // _read_twist_state_and_publish();
         // _set_target_velocities_from_subscriber();
         //_read_rpy_angles_state_and_publish();
      } catch (...) {}
    });
}

RobotDriverUnitreeH1::~RobotDriverUnitreeH1() = default;

VectorXd RobotDriverUnitreeH1::get_joint_positions()
{
   return impl_->unitree_h1_driver_->get_upper_body_joint_positions();
}

void RobotDriverUnitreeH1::set_target_joint_positions(const VectorXd& desired_joint_positions_rad)
{
   impl_->unitree_h1_driver_->set_upper_body_joint_positions(desired_joint_positions_rad);
}

VectorXd RobotDriverUnitreeH1::get_joint_velocities()
{
   return impl_->unitree_h1_driver_->get_upper_body_joint_velocities();
}

VectorXd RobotDriverUnitreeH1::get_joint_torques()
{
   return impl_->unitree_h1_driver_->get_upper_body_joint_torques();
}

void RobotDriverUnitreeH1::connect()
{
   impl_->unitree_h1_driver_->connect();
}

void RobotDriverUnitreeH1::disconnect()
{
   impl_->unitree_h1_driver_->disconnect();
}

void RobotDriverUnitreeH1::initialize()
{
   impl_->unitree_h1_driver_->initialize();
}

void RobotDriverUnitreeH1::deinitialize()
{
   impl_->unitree_h1_driver_->deinitialize();
}

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

void RobotDriverUnitreeH1::set_target_base_orientation(const DQ& r)
{
   // Not implemented - unsure if the H1 supports this action
   (void)r;
}

void RobotDriverUnitreeH1::set_target_base_height(const double& base_height)
{
   // Note that this function presumes the input, base_height, is between 0 and 100, denoting a 
   // percentage of the configured standing height range, rather than an absolute number.
   // This may need to change to maintain compatibility with other drivers.
   impl_->unitree_h1_driver_->set_stand_height_percent(base_height);
}

// ###########################################
// Additional Functions
// ###########################################

void RobotDriverUnitreeH1::_read_imu_state_and_publish()
{
    sensor_msgs::msg::Imu ros_msg_imu;
    ros_msg_imu.header.stamp = node_->get_clock()->now();

    geometry_msgs::msg::PoseStamped ros_msg_pose;
    ros_msg_pose.header.stamp = node_->get_clock()->now();

    DQ orientation = impl_->unitree_h1_driver_->get_IMU_orientation();

    if (is_unit(orientation))
    {
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
}