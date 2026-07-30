/*
# (C) Copyright 2024-2026 Adorno-Lab software developments
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

#pragma once
#include <dqrobotics/DQ.h>
#include <memory>

using namespace DQ_robotics;
using namespace Eigen;

class DriverUnitreeH1
{
    private:

    class Impl;
    std::shared_ptr<Impl> impl_;

    enum class STATUS{
        IDLE,
        CONNECTED,
        INITIALIZED,
        DEINITIALIZED,
        DISCONNECTED,
    };
    STATUS current_status_{STATUS::IDLE};
    std::string status_msg_;

    enum JOINT_INDEX {
        // Right leg
        kRightHipYaw = 8,
        kRightHipRoll = 0,
        kRightHipPitch = 1,
        kRightKnee = 2,
        kRightAnkle = 11,

        // Left leg
        kLeftHipYaw = 7,
        kLeftHipRoll = 3,
        kLeftHipPitch = 4,
        kLeftKnee = 5,
        kLeftAnkle = 10,

        // Waist
        kWaistYaw = 6,

        kNotUsedJoint = 9, // Why there is an unused joint parameter i have no idea

        // Right arm
        kRightShoulderPitch = 12,
        kRightShoulderRoll = 13,
        kRightShoulderYaw = 14,
        kRightElbow = 15, 
        // Left arm
        kLeftShoulderPitch = 16,
        kLeftShoulderRoll = 17,
        kLeftShoulderYaw = 18,
        kLeftElbow = 19,

    };

    std::array<JOINT_INDEX, 9> upper_body_joints_ = {
        JOINT_INDEX::kLeftShoulderRoll,  JOINT_INDEX::kLeftShoulderPitch,
        JOINT_INDEX::kLeftShoulderYaw,    JOINT_INDEX::kLeftElbow,
        JOINT_INDEX::kRightShoulderRoll, JOINT_INDEX::kRightShoulderPitch,
        JOINT_INDEX::kRightShoulderYaw,   JOINT_INDEX::kRightElbow, JOINT_INDEX::kWaistYaw
    };

    std::array<JOINT_INDEX, 19> robot_joints_ = {
        JOINT_INDEX::kLeftShoulderRoll,  JOINT_INDEX::kLeftShoulderPitch,
        JOINT_INDEX::kLeftShoulderYaw,    JOINT_INDEX::kLeftElbow,
        JOINT_INDEX::kRightShoulderRoll, JOINT_INDEX::kRightShoulderPitch,
        JOINT_INDEX::kRightShoulderYaw,   JOINT_INDEX::kRightElbow, 
        JOINT_INDEX::kWaistYaw,
        JOINT_INDEX::kRightHipRoll, JOINT_INDEX::kRightHipPitch,
        JOINT_INDEX::kRightHipYaw, JOINT_INDEX::kRightKnee, JOINT_INDEX::kRightAnkle,
        JOINT_INDEX::kRightHipRoll, JOINT_INDEX::kRightHipPitch,
        JOINT_INDEX::kRightHipYaw, JOINT_INDEX::kRightKnee, JOINT_INDEX::kRightAnkle,
    };

    public:

    DriverUnitreeH1() = delete;
    DriverUnitreeH1(const DriverUnitreeH1&) = delete;
    DriverUnitreeH1& operator= (const DriverUnitreeH1&) = delete;
    DriverUnitreeH1(std::string network_interface);

    void connect();
    void initialize();
    void deinitialize();
    void disconnect();

    VectorXd get_upper_body_joint_positions() const;
    VectorXd get_upper_body_joint_velocities() const;
    VectorXd get_upper_body_joint_torques() const;
    VectorXd get_upper_body_joint_temperatures() const;

    VectorXd get_torso_velocity() const;

    DQ get_IMU_orientation() const;
    VectorXd get_gyroscope_data() const;
    VectorXd get_accelerometer_data() const;
    VectorXd get_Euler_angles() const;
    int get_IMU_temperature() const;

    VectorXd get_battery_temperatures() const;
    int get_battery_state_of_charge() const;

    void set_upper_body_joint_positions(const VectorXd& desired_joint_positions_rad);
    void set_upper_body_joint_velocities(const VectorXd& desired_joint_velocities_rad_per_sec);
    void set_upper_body_joint_torques(const VectorXd& desired_joint_torques_Nm);
    
    void set_torso_velocity(const VectorXd& desired_torso_velocity_mps_radps);

    private:

    void set_all_upper_body_joint_position_commands_(const VectorXd& target_positions_rad);
    void set_all_upper_body_joint_velocity_commands_(const VectorXd& target_velocities_rad_per_sec);
    void set_all_upper_body_joint_torque_commands_(const VectorXd& target_torques_Nm);
    
};