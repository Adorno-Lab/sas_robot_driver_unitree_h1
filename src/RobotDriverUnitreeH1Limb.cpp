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
#                    src/RobotDriverUnitreeG1Limb.cpp written by Juan Jose 
#                    Quiroz Omana (juanjose.quirozomana@manchester.ac.uk) for the 
#                    unitree G1 driver package:
#                    https://github.com/Adorno-Lab/sas_robot_driver_unitree_g1/tree/main
#
# ################################################################
*/

#include "RobotDriverUnitreeH1Limb.hpp"
#include <stdexcept>

namespace sas
{

// ###########################################
// Constructor
// ###########################################

RobotDriverUnitreeH1Limb::RobotDriverUnitreeH1Limb(const std::shared_ptr<DriverUnitreeH1> &hardware,
                                                   const DriverUnitreeLowState::LIMB &limb,
                                                   const std::string &name,
                                                   const std::tuple<Eigen::VectorXd, Eigen::VectorXd> &joint_limits,
                                                   const std::shared_ptr<ShutdownSignaler> &shutdown_signaler):
    RobotDriver(shutdown_signaler),
    hardware_(hardware),
    limb_(limb),
    num_joints_(std::get<0>(joint_limits).size()),
    name_(name)
{
    set_joint_limits(joint_limits);
}

// ###########################################
// State Getters
// ###########################################

Eigen::VectorXd RobotDriverUnitreeH1Limb::get_joint_positions()
{
    Eigen::VectorXd values = hardware_->get_joint_positions(limb_);

    // Sanity check dimensions
    if (values.size() != num_joints_){
        throw std::runtime_error("[RobotDriverUnitreeH1Limb::get_joint_positions] "+name_+" driver expected "+std::to_string(num_joints_)+" values but got "+std::to_string(values.size()));
    }

    return values;
}

Eigen::VectorXd RobotDriverUnitreeH1Limb::get_joint_velocities()
{
    Eigen::VectorXd values = hardware_->get_joint_velocities(limb_);

    // Sanity check dimensions
    if (values.size() != num_joints_){
        throw std::runtime_error("[RobotDriverUnitreeH1Limb::get_joint_velocities] "+name_+" driver expected "+std::to_string(num_joints_)+" values but got "+std::to_string(values.size()));
    }

    return values;
}

Eigen::VectorXd RobotDriverUnitreeH1Limb::get_joint_torques()
{
    Eigen::VectorXd values = hardware_->get_joint_torques(limb_);

    // Sanity check dimensions
    if (values.size() != num_joints_){
        throw std::runtime_error("[RobotDriverUnitreeH1Limb::get_joint_torques] "+name_+" driver expected "+std::to_string(num_joints_)+" values but got "+std::to_string(values.size()));
    }

    return values;
}

// ###########################################
// Target Setters
// ###########################################

void RobotDriverUnitreeH1Limb::set_target_joint_positions(const Eigen::VectorXd &target_joint_positions)
{

    // Sanity check dimensions
    if (target_joint_positions.size() != num_joints_){
        throw std::runtime_error("[RobotDriverUnitreeH1Limb::set_target_joint_positions] "+name_+" driver expected "+std::to_string(num_joints_)+" values but got "+std::to_string(target_joint_positions.size()));
    }

    hardware_->set_joint_positions(target_joint_positions, limb_);
}


// ###########################################
// Lifecycle Functions
// ###########################################

// These functions need to exist to maintain comatability with the base class.
// However, the lifecycle of the hardware belongs to RobotDriverUnitreeH1. Therefore,
// these functions just don't do anything.
void RobotDriverUnitreeH1Limb::connect()
{

}

void RobotDriverUnitreeH1Limb::disconnect()
{

}

void RobotDriverUnitreeH1Limb::initialize()
{

}

void RobotDriverUnitreeH1Limb::deinitialize()
{

}

}