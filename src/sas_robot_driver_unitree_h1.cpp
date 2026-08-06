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

class RobotDriverUnitreeH1::Impl
{
public:
//    std::shared_ptr<DriverUnitreeH1> unitree_h1_driver_; // needs to have it's arguments provided

   Impl() = default;
};

RobotDriverUnitreeH1::RobotDriverUnitreeH1()
  : LeggedRobotDriver(static_cast<std::atomic_bool*>(nullptr)),
    impl_(std::make_unique<Impl>())
{
}

RobotDriverUnitreeH1::~RobotDriverUnitreeH1() = default;

VectorXd RobotDriverUnitreeH1::get_joint_positions()
{
   return VectorXd::Zero(0);
}

void RobotDriverUnitreeH1::set_target_joint_positions(const VectorXd& desired_joint_positions_rad)
{
   (void)desired_joint_positions_rad;
}

VectorXd RobotDriverUnitreeH1::get_joint_velocities()
{
   return VectorXd::Zero(0);
}

VectorXd RobotDriverUnitreeH1::get_joint_torques()
{
   return VectorXd::Zero(0);
}

void RobotDriverUnitreeH1::connect()
{
}

void RobotDriverUnitreeH1::disconnect()
{
}

void RobotDriverUnitreeH1::initialize()
{
}

void RobotDriverUnitreeH1::deinitialize()
{
}

void RobotDriverUnitreeH1::set_target_twist(const DQ& twist)
{
   (void)twist;
}

void RobotDriverUnitreeH1::set_target_base_orientation(const DQ& r)
{
   (void)r;
}

void RobotDriverUnitreeH1::set_target_base_height(const double& base_height)
{
   (void)base_height;
}

}