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

#pragma once

#include <array>
#include <memory>
#include <string>

#include <dqrobotics/DQ.h>

#include <unitree_drivers/DriverUnitreeLocoClient.h>
#include <unitree_drivers/DriverUnitreeArmSDK.h>
#include <unitree_drivers/DriverUnitreeLowState.h>

#include <marinholab/sas/core/sas_shutdown_signaler.hpp>

using namespace DQ_robotics;
using namespace Eigen;

/**
 * @class DriverUnitreeH1
 * @brief Lower level class that handles communication with the robot firmware.
 *
 * This class handles communication with the robot firmware via the driver components
 * provided by the [unitree_drivers package](https://github.com/Adorno-Lab/unitree_drivers/tree/main). 
 * Usage examples are available under `src/standalone_scripts/`. 
 */
class DriverUnitreeH1
{
private:
    
    DriverUnitreeArmSDK arm_sdk_; 
    DriverUnitreeLocoClient loco_client_;
    DriverUnitreeLowState low_state_;

    std::string network_interface_ = "Not Initialized";
    bool enter_damping_mode_on_deinit_ = false;
    bool is_dummy_ = false;

    static constexpr float comms_timeout_sec_ = 1.0f;
    static constexpr std::chrono::duration<double> comms_timeout_chrono_sec_ {comms_timeout_sec_};

protected:
    enum class MODE
    {
        POSITION_CONTROLLED,
        VELOCITY_CONTROLLED,
        TORQUE_CONTROLLED
    };
    MODE current_mode_{MODE::POSITION_CONTROLLED};
    std::string current_mode_string_ = "position_controlled";

    enum class STATUS
    {
        IDLE,
        CONNECTED,
        INITIALIZED,
        DEINITIALIZED,
        DISCONNECTED,
    };
    STATUS current_status_{STATUS::IDLE};

    enum JOINT_INDEX
    {
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

        // This slot is reserved for setting the "weight" parameter and is not a real actuator index.
        kNotUsedJoint = 9,

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

    enum FSM_ID
    {
        START = 204,
        DAMP = 1,
    };

    std::array<JOINT_INDEX, 9> upper_body_joints_ = {
        JOINT_INDEX::kLeftShoulderPitch, JOINT_INDEX::kLeftShoulderRoll,
        JOINT_INDEX::kLeftShoulderYaw,    JOINT_INDEX::kLeftElbow,
        JOINT_INDEX::kRightShoulderPitch, JOINT_INDEX::kRightShoulderRoll,
        JOINT_INDEX::kRightShoulderYaw,   JOINT_INDEX::kRightElbow, 
        JOINT_INDEX::kWaistYaw
    };

    std::array<JOINT_INDEX, 19> robot_joints_ = {
        JOINT_INDEX::kLeftShoulderPitch, JOINT_INDEX::kLeftShoulderRoll,
        JOINT_INDEX::kLeftShoulderYaw,    JOINT_INDEX::kLeftElbow,
        JOINT_INDEX::kRightShoulderPitch, JOINT_INDEX::kRightShoulderRoll,
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
    DriverUnitreeH1& operator=(const DriverUnitreeH1&) = delete;

    DriverUnitreeH1(std::string network_interface, std::string control_mode, bool ENTER_DAMPING_MODE_ON_DEINIT, bool DUMMY_MODE, const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler);
    DriverUnitreeH1(std::string network_interface, std::string control_mode, const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler);
    DriverUnitreeH1(std::string network_interface, const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler);

    void connect();
    void initialize();
    void deinitialize();
    void disconnect();

    void change_control_mode(const std::string& new_mode);

    VectorXd get_upper_body_joint_positions();
    VectorXd get_upper_body_joint_velocities() const;
    VectorXd get_upper_body_joint_torques() const;
    VectorXd get_joint_temperatures() const;
    VectorXd get_torso_velocity() const;
    VectorXd get_battery_temperatures() const;

    DriverUnitreeLowState::IMUData get_IMU_data() const;

    float get_stand_height_percent() const;

    int get_battery_state_of_charge() const;

    std::string get_control_mode();

    void set_upper_body_joint_positions(const VectorXd& desired_joint_positions_rad);
    void set_upper_body_joint_velocities(const VectorXd& desired_joint_velocities_rad_per_sec);
    void set_upper_body_joint_torques(const VectorXd& desired_joint_torques_Nm);
    void set_torso_velocity(const VectorXd& desired_torso_velocity_mps_radps);
    void set_stand_height_percent(const float desired_height_percent);

private:

    void common_construction_tasks_(std::string network_interface, std::string control_mode, bool ENTER_DAMPING_MODE_ON_DEINIT, bool DUMMY_MODE);

};