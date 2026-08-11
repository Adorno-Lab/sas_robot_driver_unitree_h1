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
    std::string robot_name;
};


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


    //also equivalent to rclcpp::TimerBase::SharedPtr
    // std::shared_ptr<rclcpp::TimerBase> timer_;

    //Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_joint_positions_;

    // Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_FR_joint_states_;
    // Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_FL_joint_states_;
    // Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_RR_joint_states_;
    // Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_RL_joint_states_;

    Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_IMU_state_;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_IMU_orientation_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_temperatures_;
    Publisher<std_msgs::msg::Float64>::SharedPtr publisher_stand_height_percent_;
    
//    // Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_last_IMU_orientation_when_robot_stopped_;
//     Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_state_;
//     Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_battery_state_;
//     Publisher<geometry_msgs::msg::TwistStamped>:: SharedPtr publisher_high_level_velocities_state_;
//    // Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_rpy_angles_;

    // //----------------Deprecated subscription to command the robot in walking mode--------------------------//
    // Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_target_holonomic_velocities_;
    // VectorXd target_holonomic_velocities_ = VectorXd::Zero(3);
    // void _callback_target_holonomic_velocities(const std_msgs::msg::Float64MultiArray& msg);
    // bool new_target_velocities_available_{false};
    // //-------------------------------------------------------------------------------------------------------//

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

   // Subscription<sas_msgs::msg::Bool>::SharedPtr subscriber_shutdown_signal_;
   // void _callback_shutdown_signal_(const sas_msgs::msg::Bool& msg);
   // bool shutdown_signal_;

   // Subscription<sas_msgs::msg::Bool>::SharedPtr subscriber_emergency_stop_device_signal_;
   // void _callback_emergency_stop_device_signal(const sas_msgs::msg::Bool& msg);

    class Impl;
    std::unique_ptr<Impl> impl_;

    // void _initial_settings();

protected:

//     void _read_joint_states_and_publish();
    void _read_imu_state_and_publish();
    void _read_temperatures_and_publish();
    void _set_torso_velocities_from_subscriber();
    void _set_stand_height_percent_from_subscriber();
    void _read_stand_height_and_publish();
//     void _read_twist_state_and_publish();
//    //publisher_rpy_angles_ void _read_rpy_angles_state_and_publish();
//     void _read_battery_state();
//    // bool _should_shutdown() const;
//     void _set_target_velocities_from_subscriber();
//     void _set_target_stand_commands_from_subscriber();


public:

    RobotDriverUnitreeH1(const RobotDriverUnitreeH1&)=delete;
    RobotDriverUnitreeH1()=delete;
    ~RobotDriverUnitreeH1();

    RobotDriverUnitreeH1(std::shared_ptr<Node>& node,
                         const RobotDriverUnitreeH1Configuration &configuration,
                         const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);


    //----------RobotDriver----methods---------------------------------//

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    //void set_target_joint_velocities(const VectorXd& desired_joint_velocities_rad_s) override;

    VectorXd get_joint_velocities() override;
    VectorXd get_joint_torques() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

    void set_target_twist(const DQ& twist) override;
    void set_target_base_orientation(const DQ& r) override;
    void set_target_base_height(const double& base_height) override;

    // Extra methods
    void set_target_joint_velocities(const VectorXd& desired_joint_velocities_radps) override;

    void set_target_joint_torques(const VectorXd& desired_joint_torques_Nm) override;

};



}