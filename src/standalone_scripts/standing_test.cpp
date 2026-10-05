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

// This script moves the robot through a stand-height sequence to demonstrate the body-height
// command interface and verify the lower-body posture control behaviour in standalone mode.

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
    float max_height = 1.0;
    float min_height = 0.0;  
    float num_cycles = 1.0/sleep_time_sec;


//clamped between about 0.6 and 0.8

    std::cout << "    Begin motion..."<<std::endl;

    std::cout << "        Press ENTER to go to start height...";
    std::cin.get();
    float current_height_percent = driver.get_stand_height_percent();
    float target_height_percent = 50;
    float starting_height_percent = current_height_percent;
    for(int i=0; i<num_cycles; i++){
        current_height_percent = starting_height_percent + (static_cast<float>(i)/num_cycles) * (target_height_percent-starting_height_percent);
        driver.set_stand_height_percent(current_height_percent);
        std::cout << "            Height: "<<current_height_percent<<"%"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "        Press ENTER to go to up...";
    std::cin.get();
    target_height_percent = 100;
    starting_height_percent = current_height_percent;
    for(int i=0; i<num_cycles; i++){
        current_height_percent = starting_height_percent + (static_cast<float>(i)/num_cycles) * (target_height_percent-starting_height_percent);
        driver.set_stand_height_percent(current_height_percent);
        std::cout << "            Height: "<<current_height_percent<<"%"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "        Press ENTER to go to down...";
    std::cin.get();
    target_height_percent = 0;
    starting_height_percent = current_height_percent;
    for(int i=0; i<num_cycles*2; i++){
        current_height_percent = starting_height_percent + (static_cast<float>(i)/(num_cycles*2)) * (target_height_percent-starting_height_percent);
        driver.set_stand_height_percent(current_height_percent);
        std::cout << "            Height: "<<current_height_percent<<"%"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "        Press ENTER to go return to normal...";
    std::cin.get();
    target_height_percent = 50;
    starting_height_percent = current_height_percent;
    for(int i=0; i<num_cycles; i++){
        current_height_percent = starting_height_percent + (static_cast<float>(i)/num_cycles) * (target_height_percent-starting_height_percent);
        driver.set_stand_height_percent(current_height_percent);
        std::cout << "            Height: "<<current_height_percent<<"%"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

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