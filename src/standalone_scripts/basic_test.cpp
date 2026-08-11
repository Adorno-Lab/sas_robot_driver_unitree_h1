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

#include "DriverUnitreeH1.hpp"

using namespace Eigen;

int main(){
    
    // Create robot driver
    std::cout << "Creating robot driver..." << std::endl;
    DriverUnitreeH1 driver("eth0");
    std::cout << "    Done." << std::endl;

    // Connect
    std::cout << "Press ENTER to connect ...";
    std::cin.get();
    driver.connect();
    std::cout << "    Done." << std::endl;

    // Initialise
    std::cout << "Press ENTER to initialise ...";
    std::cin.get();
    driver.initialize();
    std::cout << "    Done." << std::endl;

    // The task
    std::cout << "Press ENTER to perform motion task ...";
    std::cin.get();
    
    std::cout << "    Defining variables..."<<std::endl;
    float sleep_time_sec = 0.02;
    float num_cycles = 4.0 / sleep_time_sec;
    VectorXd target_pos(9);
    target_pos << 1.571, 0.f,  1.571, 0.f,
                 -1.571, 0.f, -1.571, 0.f, 
                  0.f;
    std::cout << "    Get current joint positions..."<<std::endl;
    VectorXd starting_pos = driver.get_upper_body_joint_positions();
    VectorXd desired_torso_velocity(3);
    desired_torso_velocity<<0.2,0,0;
    std::cout << "    Begin motion..."<<std::endl;
    for(int i=0; i<num_cycles; i++){
        driver.set_torso_velocity(desired_torso_velocity);
        driver.set_upper_body_joint_positions(starting_pos + (static_cast<float>(i)/num_cycles)*(target_pos-starting_pos));
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }

    driver.set_torso_velocity(VectorXd::Zero(3));
    std::cout << "    Done." << std::endl;

    // Deinitialise
    std::cout << "Press ENTER to deinitialise ...";
    std::cin.get();
    driver.deinitialize();
    std::cout << "    Done." << std::endl;

    // Disconnect
    std::cout << "Press ENTER to disconnect ...";
    std::cin.get();
    driver.disconnect();
    std::cout << "    Done." << std::endl;
}