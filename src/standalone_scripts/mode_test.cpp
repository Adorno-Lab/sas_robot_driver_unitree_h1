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
    DriverUnitreeH1 driver("eth0", "position_controlled");
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
    
    std::cout << "    Sending position commands in position mode..."<<std::endl;
    float sleep_time_sec = 0.02;
    float num_cycles = 4.0 / sleep_time_sec;
    VectorXd target_pos(9);
    target_pos << 1.571, 0.f,  1.571, 0.f,
                 -1.571, 0.f, -1.571, 0.f, 
                  0.f;
    VectorXd starting_pos = driver.get_upper_body_joint_positions();
    for(int i=0; i<num_cycles; i++){
        driver.set_upper_body_joint_positions(starting_pos + (static_cast<float>(i)/num_cycles)*(target_pos-starting_pos));
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }
    std::cout << "    Done." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "    Sending velocity commands in position mode..."<<std::endl;
    VectorXd target_vels(9);
    target_vels << 0.f, 0.f,  0.f, 0.f,
                  0.f, 0.f,  0.f, 0.f, 
                  0.f;
    try{
        driver.set_upper_body_joint_velocities(target_vels);
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] "<<e.what()<<std::endl;
    }
    std::cout << "    Done." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "    Switching to velocity mode..."<<std::endl;
    driver.change_control_mode("velocity_controlled");
    std::cout << "    Done." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "    Sending velocity commands again..."<<std::endl;
    try{
        driver.set_upper_body_joint_velocities(target_vels);
    }
    catch (const std::exception& e){
        std::cout << "[ERROR] "<<e.what()<<std::endl;
    }
    std::cout << "    Done." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

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