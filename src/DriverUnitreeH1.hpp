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

using namespace DQ_robotics;
using namespace Eigen;

/**
 * @class DriverUnitreeH1
 * @brief High-level control interface for the Unitree H1 robot.
 *
 * This class manages the communication lifecycle, state acquisition, and command
 * dispatch for the robot's upper body, torso motion, and locomotion functions.
 * It encapsulates the underlying DDS and locomotion client interfaces and exposes
 * a small, driver-level API that can be used by higher-level controllers.
 */
class DriverUnitreeH1
{
private:
    class Impl;
    std::shared_ptr<Impl> impl_;

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
    DriverUnitreeH1& operator=(const DriverUnitreeH1&) = delete;

    /**
     * @brief Construct a driver instance with explicit network configuration and control mode.
     * @param network_interface Network interface used for Unitree DDS communication.
     * @param control_mode Active control mode, one of "position_controlled",
     *        "velocity_controlled", or "torque_controlled".
     * @param ENTER_DAMPING_MODE_ON_DEINIT If true, the robot is placed in damping mode during deinitialization.
     * @param DUMMY_MODE If true, the driver behaves in a passive dummy mode without hardware access.
     * @throws std::runtime_error if the requested control mode is not recognised.
     */
    DriverUnitreeH1(std::string network_interface, std::string control_mode, bool ENTER_DAMPING_MODE_ON_DEINIT, bool DUMMY_MODE);

    /**
     * @brief Construct a driver instance with a specific network interface and control mode.
     * @param network_interface Network interface used for Unitree DDS communication.
     * @param control_mode Active control mode, one of "position_controlled",
     *        "velocity_controlled", or "torque_controlled".
     * @throws std::runtime_error if the requested control mode is not recognised.
     */
    DriverUnitreeH1(std::string network_interface, std::string control_mode);

    /**
     * @brief Construct a driver instance using the default position-control mode.
     * @param network_interface Network interface used for Unitree DDS communication.
     */
    DriverUnitreeH1(std::string network_interface);

    /**
     * @brief Perform the common initialization tasks shared by all constructors.
     * @param network_interface Network interface used for Unitree DDS communication.
     * @param control_mode Requested control mode. Must be one of the supported values.
     * @param ENTER_DAMPING_MODE_ON_DEINIT If true, damping mode is entered on release.
     * @param DUMMY_MODE If true, the driver runs without robot communication.
     * @throws std::runtime_error if the supplied control mode is invalid.
     */
    void common_construction_tasks(std::string network_interface, std::string control_mode, bool ENTER_DAMPING_MODE_ON_DEINIT, bool DUMMY_MODE);

    /**
     * @brief Connect to the robot communication channels and locomotion server.
     * @throws std::runtime_error if the low-level state subscriber or locomotion client cannot be initialized correctly.
     */
    void connect();

    /**
     * @brief Initialize the robot after the communication channels are active.
     * @throws std::runtime_error if the driver is not connected before initialization.
     */
    void initialize();

    /**
     * @brief Safely stop locomotion and transition the robot into a deinitialized state.
     * @throws nothing; this function is designed to be safe during teardown and destructor paths.
     */
    void deinitialize();

    /**
     * @brief Close the communication channels and release the robot connection.
     * @throws nothing; this function is designed to be safe during teardown and destructor paths.
     */
    void disconnect();

    /**
     * @brief Read the current positions of the upper-body joints.
     * @return A VectorXd with one entry per upper-body joint, in radians.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    VectorXd get_upper_body_joint_positions() const;

    /**
     * @brief Read the current velocities of the upper-body joints.
     * @return A VectorXd with one entry per upper-body joint, in radians per second.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    VectorXd get_upper_body_joint_velocities() const;

    /**
     * @brief Read the estimated torques of the upper-body joints.
     * @return A VectorXd with one entry per upper-body joint, in newton-metres.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    VectorXd get_upper_body_joint_torques() const;

    /**
     * @brief Read the joint casing temperatures reported by the robot.
     * @return A VectorXd containing the measured temperatures in degrees Celsius.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    VectorXd get_joint_temperatures() const;

    /**
     * @brief Read the torso linear and angular velocity estimate.
     * @return A 3-vector of the form {vx, vy, omega}.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    VectorXd get_torso_velocity() const;

    /**
     * @brief Read the current IMU orientation as a dual quaternion.
     * @return A DQ quaternion representing the latest IMU orientation.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    DQ get_IMU_orientation() const;

    /**
     * @brief Read the current IMU gyroscope readings.
     * @return A 3-vector of angular rates in radians per second.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    VectorXd get_gyroscope_data() const;

    /**
     * @brief Read the current IMU accelerometer readings.
     * @return A 3-vector of linear acceleration in metres per second squared.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    VectorXd get_accelerometer_data() const;

    /**
     * @brief Read the current IMU Euler angles.
     * @return A 3-vector in roll-pitch-yaw order, in radians.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    VectorXd get_Euler_angles() const;

    /**
     * @brief Read the IMU temperature.
     * @return Current IMU temperature in degrees Celsius.
     * @throws std::runtime_error if the driver is not initialized or the state connection has timed out.
     */
    int get_IMU_temperature() const;

    /**
     * @brief Query the current stand height as a percentage of the configured operating range.
     * @return A percentage in the range supported by the robot's stand-height controller.
     */
    float get_stand_height_percent() const;

    /**
     * @brief Read the reported battery temperatures.
     * @return A 2-vector containing the battery temperature measurements in degrees Celsius.
     * @throws std::runtime_error if this feature is not supported by the current firmware or configuration.
     */
    VectorXd get_battery_temperatures() const;

    /**
     * @brief Query the battery state of charge.
     * @return Battery state of charge as a percentage.
     * @throws std::runtime_error if this feature is not supported by the current firmware or configuration.
     */
    int get_battery_state_of_charge() const;

    /**
     * @brief Change the active upper-body control mode at runtime.
     * @param new_mode A supported control mode string: "position_controlled", "velocity_controlled", or "torque_controlled".
     * @throws std::runtime_error if the mode string is invalid or the driver is not initialized.
     */
    void change_control_mode(const std::string& new_mode);

    /**
     * @brief Return the currently active control mode string.
     * @return The active upper-body control mode.
     * @throws std::runtime_error if the driver is not initialized.
     */
    std::string get_control_mode();

    /**
     * @brief Command all upper-body joints to the specified target angles.
     * @param desired_joint_positions_rad Desired joint angles in radians.
     * @throws std::runtime_error if the driver is not initialized, the requested mode is not position control, or the vector size is incorrect.
     */
    void set_upper_body_joint_positions(const VectorXd& desired_joint_positions_rad);

    /**
     * @brief Command all upper-body joints to the specified target velocities.
     * @param desired_joint_velocities_rad_per_sec Desired joint velocities in radians per second.
     * @throws std::runtime_error if the driver is not initialized, the requested mode is not velocity control, or the vector size is incorrect.
     */
    void set_upper_body_joint_velocities(const VectorXd& desired_joint_velocities_rad_per_sec);

    /**
     * @brief Command all upper-body joints to the specified joint torques.
     * @param desired_joint_torques_Nm Desired joint torques in newton-metres.
     * @throws std::runtime_error if the driver is not initialized, the requested mode is not torque control, or the vector size is incorrect.
     */
    void set_upper_body_joint_torques(const VectorXd& desired_joint_torques_Nm);

    /**
     * @brief Command the torso velocity in the robot frame.
     * @param desired_torso_velocity_mps_radps A 3-vector containing {vx, vy, omega}.
     * @throws std::runtime_error if the driver is not initialized or the supplied vector is not size 3.
     */
    void set_torso_velocity(const VectorXd& desired_torso_velocity_mps_radps);

    /**
     * @brief Set the robot standing height as a percentage of the configured range.
     * @param desired_height_percent Height setpoint expressed as a percentage between the stand-height limits.
     */
    void set_stand_height_percent(const float desired_height_percent);

private:
    /**
     * @brief Populate the motor command buffer for all upper-body joint position targets.
     * @param target_positions_rad Desired positions in radians.
     */
    void set_all_upper_body_joint_position_commands_(const VectorXd& target_positions_rad);

    /**
     * @brief Populate the motor command buffer for all upper-body joint velocity targets.
     * @param target_velocities_rad_per_sec Desired velocities in radians per second.
     */
    void set_all_upper_body_joint_velocity_commands_(const VectorXd& target_velocities_rad_per_sec);

    /**
     * @brief Populate the motor command buffer for all upper-body joint torque targets.
     * @param target_torques_Nm Desired torques in newton-metres.
     */
    void set_all_upper_body_joint_torque_commands_(const VectorXd& target_torques_Nm);

    /**
     * @brief Command all upper-body joints to a passive damping state.
     */
    void damp_all_upper_body_joints_();

    /**
     * @brief Safely initialize the upper-body joints and ramp their gains into the active control state.
     */
    void safely_start_upper_body_joints_();

    /**
     * @brief Safely return the upper-body joints to a passive damping state.
     */
    void safely_stop_upper_body_joints_();
};