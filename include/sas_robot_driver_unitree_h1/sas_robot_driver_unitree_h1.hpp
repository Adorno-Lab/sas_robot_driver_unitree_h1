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
#   based on the version of this file in the unitree B1 driver by Juan Jose Quiroz Omana
#   https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1/tree/main
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
 * @brief ROS 2 driver wrapper for the Unitree H1 robot backend.
 *
 * This class adapts the low-level Unitree H1 control interface to the SAS robot
 * driver API and exposes the required ROS 2 publishers, subscribers, and
 * lifecycle methods used by higher-level motion and state-estimation tooling.
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

    /**
     * @brief Construct the ROS 2 driver wrapper for a given robot configuration.
     * @param node Shared ROS 2 node used to create publishers and subscriptions.
     * @param configuration Driver configuration containing network and control settings.
     * @param shutdown_signaler Signal source used by the base robot driver lifecycle.
     */
    RobotDriverUnitreeH1(std::shared_ptr<Node>& node,
                         const RobotDriverUnitreeH1Configuration &configuration,
                         const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    /**
     * @brief Read the current joint positions reported by the robot backend.
     * @return A VectorXd containing the joint positions in radians.
     */
    VectorXd get_joint_positions() override;

    /**
     * @brief Send target joint positions to the active controller.
     * @param desired_joint_positions_rad Desired joint angles in radians.
     */
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    /**
     * @brief Read the current joint velocities reported by the robot backend.
     * @return A VectorXd containing the joint velocities in radians per second.
     */
    VectorXd get_joint_velocities() override;

    /**
     * @brief Read the latest estimated joint torques.
     * @return A VectorXd containing the joint torques in newton-metres.
     */
    VectorXd get_joint_torques() override;

    /**
     * @brief Connect the driver to the Unitree robot backend and establish the ROS 2 interfaces.
     */
    void connect() override;

    /**
     * @brief Disconnect the driver from the robot backend and release communication resources.
     */
    void disconnect() override;

    /**
     * @brief Initialize the robot after a successful connection is established.
     */
    void initialize() override;

    /**
     * @brief Safely deinitialize the robot and stop motion before teardown.
     */
    void deinitialize() override;

    /**
     * @brief Set a target body twist for the robot.
     * @param twist A dual-quaternion representation of the target twist to apply.
     */
    void set_target_twist(const DQ& twist) override;

    /**
     * @brief Set the target base orientation.
     * @param r Target base orientation expressed as a dual quaternion.
     */
    void set_target_base_orientation(const DQ& r) override;

    /**
     * @brief Set the target stand height as a percentage of the robot's configured operating range.
     * @param base_height Desired stand-height percentage.
     */
    void set_target_base_height(const double& base_height) override;

    /**
     * @brief Send target joint velocities to the robot backend.
     * @param desired_joint_velocities_radps Desired joint velocities in radians per second.
     */
    void set_target_joint_velocities(const VectorXd& desired_joint_velocities_radps) override;

    /**
     * @brief Send target joint torques to the robot backend.
     * @param desired_joint_torques_Nm Desired joint torques in newton-metres.
     */
    void set_target_joint_torques(const VectorXd& desired_joint_torques_Nm) override;
};

}