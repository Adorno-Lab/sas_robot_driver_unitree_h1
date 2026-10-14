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
#                    src/RobotDriverUnitreeG1Limb.h written by Juan Jose 
#                    Quiroz Omana (juanjose.quirozomana@manchester.ac.uk) for the 
#                    unitree G1 driver package:
#                    https://github.com/Adorno-Lab/sas_robot_driver_unitree_g1/tree/main
#
# ################################################################
*/

#pragma once
#include <memory>
#include <tuple>
#include <sas_core/sas_robot_driver.hpp>
#include "DriverUnitreeH1.h"

namespace sas
{

/**
 * @brief The RobotDriverUnitreeH1Limb class is a RobotDriver view of one limb of the Unitree H1
 *        (a leg, the waist, or an arm), on top of the DriverUnitreeH1 shared by every limb.
 *
 * The joint layout of each limb is the one of unitree_drivers' DriverUnitreeLowState::LIMB. The
 * lifecycle (connect(), initialize(), ...) belongs to RobotDriverUnitreeH1, so it is a no-op here.
 */
class RobotDriverUnitreeH1Limb: public RobotDriver
{
private:
    std::shared_ptr<DriverUnitreeH1> hardware_;
    DriverUnitreeLowState::LIMB limb_;
    int num_joints_;
    std::string name_;

public:
    RobotDriverUnitreeH1Limb()=delete;
    RobotDriverUnitreeH1Limb(const RobotDriverUnitreeH1Limb&)=delete;

    RobotDriverUnitreeH1Limb(const std::shared_ptr<DriverUnitreeH1>& hardware,
                             const DriverUnitreeLowState::LIMB& limb,
                             const std::string& name,
                             const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& joint_limits,
                             const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    Eigen::VectorXd get_joint_positions() override;

    Eigen::VectorXd get_joint_velocities() override;

    Eigen::VectorXd get_joint_torques() override;

    void set_target_joint_positions(const Eigen::VectorXd& target_joint_positions) override;

    void connect() override;
    void disconnect() override;
    void initialize() override;
    void deinitialize() override;
};

}