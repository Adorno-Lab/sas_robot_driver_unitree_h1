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
#include "RobotDriverUnitreeH1Limb.hpp"

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
   {"left_arm", LIMB::LEFT_ARM,
      {"LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow"},
      {-2.87, -0.34, -1.30, -1.25},
      { 2.87,  3.11,  4.45,  2.61}},
   {"right_arm", LIMB::RIGHT_ARM,
      {"RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow"},
      {-2.87, -3.11, -4.45, -1.25},
      { 2.87,  0.34,  1.30,  2.61}},
   {"waist", LIMB::WAIST,
      {"WaistYaw"},
      {-2.35},
      { 2.35}},
   {"left_leg", LIMB::LEFT_LEG,
      {"LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnkle"},
      {-3.14, -0.43, -0.43, -0.26, -0.87},
      { 3.14,  0.43,  0.43,  2.05,  0.52}},
   {"right_leg", LIMB::RIGHT_LEG,
      {"RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnkle"},
      {-3.14, -0.43, -0.43, -0.26, -0.87},
      { 3.14,  0.43,  0.43,  2.05,  0.52}},
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
   std::vector<LimbEntry> limbs_;

   Impl() = default;
};

// ###########################################
// Constructor and Lifecycle Functions
// ###########################################

RobotDriverUnitreeH1::RobotDriverUnitreeH1(const RobotDriverUnitreeH1Configuration &configuration,
                                           const std::shared_ptr<ShutdownSignaler> &shutdown_signaler)
    :LeggedRobotDriver{shutdown_signaler},
    configuration_{configuration}
{
   impl_ = std::make_unique<RobotDriverUnitreeH1::Impl>();

   impl_->hardware_ = std::make_shared<DriverUnitreeH1>(configuration_.network_interface, 
                                                        configuration_.mode, 
                                                        configuration_.ENTER_DAMPING_MODE_ON_DEINIT,
                                                        configuration_.DUMMY_MODE,
                                                        shutdown_signaler);

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
 * @brief Read the latest imu orientation measurement.
 * @return A DQ containing the most recent orientation.
 */
DQ RobotDriverUnitreeH1::get_orientation()
{
   DriverUnitreeLowState::IMUData imu_data = impl_->hardware_->get_IMU_data();

   // If the data is invalid (i.e., it did not come from a real measurement) then stop the process
   // right here, return zero.
   if (!imu_data.valid){
        return DQ(0);
   }

   DQ orientation = DQ(
      imu_data.quaternion.at(0), //w
      imu_data.quaternion.at(1), //x
      imu_data.quaternion.at(2), //y
      imu_data.quaternion.at(3));//z
      
   // Likewise, return zero if the DQ has norm 0, dont try to normalise it because this will cause problems.
   if(orientation.vec4().norm() == 0.0){
      return DQ(0);
   }
   
   // Otherwise, normalise and return.
   orientation.normalize();

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

   return linear_acceleration;

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

/**
 * @brief Get a vector of bools describing whether each limb is commandable in the current mode.
 * @return A the vector of bools described above, in the order of the limbs in impl_->limbs_.
 */
std::vector<bool> RobotDriverUnitreeH1::get_commandable_limbs() const
{
   // The arms and waist are commandable in all modes, while the legs are never commandable,
   // so we dont have to consider the current high level mode here.
   std::vector<bool> commandable;
   commandable.reserve(impl_->limbs_.size());

   for (const auto& limb : impl_->limbs_)
   {
      commandable.push_back(
         limb.name == "left_arm" ||
         limb.name == "right_arm" ||
         limb.name == "waist");
   }

   return commandable;
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
   impl_->hardware_->set_torso_velocity(VectorXd::Zero(3));
   
   if(mode==HIGH_LEVEL_MODE::STANDING){
      throw std::runtime_error("[RobotDriverUnitreeH1::_set_high_level_mode] Unitree H1 does not support standing mode!"); 
   }

   // Then just return, because we don't actually need to do anything.
   return;
   
}

/**
 * @brief Set the target torso twist.
 * @param twist Desired twist represented as a dual quaternion twist.
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

} // End of namespace sas