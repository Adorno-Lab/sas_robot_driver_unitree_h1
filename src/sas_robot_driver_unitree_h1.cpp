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
   std::shared_ptr<DriverUnitreeH1> unitree_h1_driver_;

   Impl() = default;
};

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

    impl_->unitree_h1_driver_ = std::make_shared<DriverUnitreeH1>("eth0", "position_controlled", configuration_.ENTER_DAMPING_MODE_ON_DEINIT);
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
   // Pull elements from twist and call set_torso_velocity
   (void)twist;
}

void RobotDriverUnitreeH1::set_target_base_orientation(const DQ& r)
{
   // Not sure if the H1 can do this one
   (void)r;
}

void RobotDriverUnitreeH1::set_target_base_height(const double& base_height)
{
   // Need to decide whether to keep percentage model or not.
   (void)base_height;
}

}