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
#   Acknowledgement: This file was adapted from the file 
#                    src/sas_robot_driver_unitree_b1.hpp written by Juan Jose 
#                    Quiroz Omana (juanjose.quirozomana@manchester.ac.uk) for the 
#                    unitree B1 driver package:
#                    https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1/tree/main
#
# ################################################################
*/

#pragma once
#include <atomic>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_msgs/msg/watchdog_trigger.hpp>
#include <sas_msgs/msg/bool.hpp>
#include <sas_core/sas_robot_driver.hpp>
#include <sas_tools/LeggedRobotDriver.hpp>

//using namespace Eigen;

using namespace rclcpp;

namespace sas
{

struct RobotDriverUnitreeH1Configuration
{
    std::string network_interface;
    std::string mode;
    bool ENTER_DAMPING_MODE_ON_DEINIT;
    bool DUMMY_MODE;
    std::string robot_name;
};

/**
 * @class RobotDriverUnitreeH1
 * @brief Higher level class that wraps `DriverUnitreeH1` and provides a ROS interface.
 *
 * This class wraps `DriverUnitreeH1` and provides a higher level ROS interface
 * that uses the [Smart Arm Stack](https://smartarmstack.github.io/) (SAS) framework.
 * 
 * Users are not intended to call the functions of this class directly. Instead, users
 * wishing for a ROS2/SAS interface should launch the drivers as described in the ReadMe
 * or the main docs page under "How to use the hardware drivers", and then interact with
 * the driver by publishing to and reading from the relevant ROS2 topics.
 * 
 * Topics include:
 *     - `/sas_h1/<robot_name>/get/joint_states` --- Subscribe to get the robot's current joint positions, velocities and torques
 *     - `/sas_h1/<robot_name>/get/IMU_state` --- Subscribe to get the most recent IMU reading (orientation, gyroscope, accelerometer information)
 *     - `/sas_h1/<robot_name>/get/imu_orientation` --- Subscribe to get the robot's current orientation (extracted from above)
 *     - `/sas_h1/<robot_name>/get/stand_height_percent` --- Subscribe to get the robot's current stand height as a percentage of the configured range
 *     - `/sas_h1/<robot_name>/get/temperatures` --- Subscribe to get the robot's current joint temperatures
 *     - `/sas_h1/<robot_name>/set/target_joint_forces` --- Publish to set new target joint forces (if in torque controlled mode)
 *     - `/sas_h1/<robot_name>/set/target_joint_positions` --- Publish to set new target joint positions (if in position controlled mode)
 *     - `/sas_h1/<robot_name>/set/target_joint_velocities` --- Publish to set new target joint velocities (if in velocity controlled mode)
 *     - `/sas_h1/<robot_name>/set/control_mode` --- Publish to set new control mode ("position_controlled", "velocity_controlled" or "torque_controlled")
 *     - `/sas_h1/<robot_name>/set/stand_height_percent` --- Publish to set new stand height as a percentage of the configured range
 *     - `/sas_h1/<robot_name>/set/target_twist` --- Publish to set new target torso twist
 *  
 * Usage examples are available under `src/ROS2_scripts/`. 
 */
class RobotDriverUnitreeH1: public LeggedRobotDriver
{
protected:
    std::string topic_prefix_;
    RobotDriverUnitreeH1Configuration configuration_;
    std::shared_ptr<rclcpp::Node> node_;

private:
    double timer_period_;
    int print_count_;
    sas::Clock clock_;

    Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_IMU_state_;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_IMU_orientation_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_temperatures_;
    Publisher<std_msgs::msg::Float64>::SharedPtr publisher_stand_height_percent_;

    Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_target_twist_;
    VectorXd target_twist_ = VectorXd::Zero(6);
    void _callback_target_twist(const geometry_msgs::msg::TwistStamped& msg);
    bool new_target_twist_available_{false};

    Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_target_stand_height_percent_;
    double target_stand_height_percent_ = 50;
    void _callback_target_stand_height_percent(const std_msgs::msg::Float64& msg);
    bool new_target_stand_height_percent_available_{false};

    Subscription<std_msgs::msg::String>::SharedPtr subscriber_set_control_mode_;
    void _callback_set_control_mode(const std_msgs::msg::String& msg);

    class Impl;
    std::unique_ptr<Impl> impl_;

protected:
    void _read_imu_state_and_publish();
    void _read_temperatures_and_publish();
    void _set_torso_velocities_from_subscriber();
    void _set_stand_height_percent_from_subscriber();
    void _read_stand_height_and_publish();

public:
    RobotDriverUnitreeH1(const RobotDriverUnitreeH1&)=delete;
    RobotDriverUnitreeH1()=delete;
    ~RobotDriverUnitreeH1();

    RobotDriverUnitreeH1(std::shared_ptr<Node>& node,
                         const RobotDriverUnitreeH1Configuration &configuration,
                         const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);


    void connect() override;
    void disconnect() override;
    void initialize() override;
    void deinitialize() override;

    VectorXd get_joint_positions() override;
    VectorXd get_joint_velocities() override;
    VectorXd get_joint_torques() override;

    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;
    void set_target_twist(const DQ& twist) override;
    void set_target_base_orientation(const DQ& r) override;
    void set_target_base_height(const double& base_height) override;
    void set_target_joint_velocities(const VectorXd& desired_joint_velocities_radps) override;
    void set_target_joint_torques(const VectorXd& desired_joint_torques_Nm) override;
};

}