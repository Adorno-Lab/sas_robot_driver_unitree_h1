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

#include <chrono>
#include <iostream>
#include <thread> 

#include <Eigen/Core>

#include <marinholab/sas/core/sas_shutdown_signaler.hpp>

#include "DriverUnitreeH1.hpp"

using namespace Eigen;

// This script commands a sustained upper-body joint-velocity profile to demonstrate the
// standalone velocity-control path and confirm that the H1 driver can track commanded motion.

int main(){
    
    // Create robot driver
    std::cout << "Creating robot driver..." << std::endl;
    static std::shared_ptr<marinholab::sas::core::ShutdownSignaler> shutdown_signaler = std::make_shared<marinholab::sas::core::ShutdownSignaler>();
    DriverUnitreeH1 driver("eth0", shutdown_signaler);
    std::cout << "    Done." << std::endl;

    // Connect
    std::cout << "Press ENTER to connect ...";
    std::cin.get();
    driver.connect();
    std::cout << "    Done." << std::endl;

    // Initialize
    std::cout << "Press ENTER to initialize ...";
    std::cin.get();
    driver.initialize();
    std::cout << "    Done." << std::endl;

    // The task
    std::cout << "Press ENTER to perform motion task ...";
    std::cin.get();
    
    std::cout << "    Defining variables..."<<std::endl;
    float sleep_time_sec = 0.02;
    float num_cycles = 4.0 / sleep_time_sec;
    VectorXd desired_velocities(19);
    desired_velocities << 0.f, 0.f, 0.f, 0.f,
                          0.f, 0.f, 0.f, 0.5,
                          0.f;

    std::cout << "    Begin motion..."<<std::endl;
    for(int i=0; i<num_cycles; i++){
        driver.set_upper_body_joint_velocities(desired_velocities);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    driver.set_upper_body_joint_velocities(VectorXd::Zero(19));
    std::cout << "    Done." << std::endl;

    // Deinitialize
    std::cout << "Press ENTER to deinitialize ...";
    std::cin.get();
    driver.deinitialize();
    std::cout << "    Done." << std::endl;

    // Disconnect
    std::cout << "Press ENTER to disconnect ...";
    std::cin.get();
    driver.disconnect();
    std::cout << "    Done." << std::endl;
}