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
#include <string>
#include <thread> 

#include <Eigen/Core>

#include <marinholab/sas/core/sas_shutdown_signaler.hpp>

#include "DriverUnitreeH1.hpp"


using namespace Eigen;

// This script establishes a basic connection to the H1 driver, initializes the robot,
// and performs a several movements of the base and arms. Used as an example of how to
// use the standalone C++ interface, and as a basic test to verify that the driver works
// as expected after changes.

int main(int argc, char * argv[]){

    // Determine if the user intends to use the dummy driver or the real robot
    const bool is_dummy = argc == 2 && std::string(argv[1]) == "--dummy";
    if (argc > 2 || (argc == 2 && !is_dummy)) {
        std::cerr << "Usage: " << argv[0] << " [--dummy]" << std::endl;
        return 1;
    }
    const std::string network_interface = is_dummy ? "lo" : "eth0";
    
    // Create robot driver
    std::cout << "Creating robot driver..." << std::endl;
    static std::shared_ptr<marinholab::sas::core::ShutdownSignaler> shutdown_signaler = std::make_shared<marinholab::sas::core::ShutdownSignaler>();
    DriverUnitreeH1 driver(network_interface, "position_controlled", false, is_dummy, shutdown_signaler);
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

    // Define some variables for the motions below
    float sleep_time_sec = 0.02;
    float num_movement_cycles = 4.0 / sleep_time_sec;
    float num_stand_height_cycles = 1.0 / sleep_time_sec;
    float target_height, current_height, starting_height;
    VectorXd target_joint_pos(9), current_joint_pos(9), starting_joint_pos(9);
    VectorXd desired_torso_velocity(3);

    // Raise the standing height
    std::cout << "Press ENTER to go to up...";
    std::cin.get();
    target_height = 0.8;
    starting_height = driver.get_stand_height();
    for(int i=0; i<num_stand_height_cycles; i++){
        current_height = starting_height + (static_cast<float>(i)/num_stand_height_cycles) * (target_height-starting_height);
        driver.set_stand_height(current_height);
        std::cout << "    Height: "<<current_height<<" m"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }
    std::cout << "    Done." << std::endl;

    // Lower the standing height
    std::cout << "Press ENTER to go to down...";
    std::cin.get();
    target_height = 0.6;
    starting_height = driver.get_stand_height();
    for(int i=0; i<num_stand_height_cycles*2; i++){
        current_height = starting_height + (static_cast<float>(i)/(num_stand_height_cycles*2)) * (target_height-starting_height);
        driver.set_stand_height(current_height);
        std::cout << "    Height: "<<current_height<<" m"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }
    std::cout << "    Done." << std::endl;

    // Return to a normal height
    std::cout << "Press ENTER to go return to normal...";
    std::cin.get();
    target_height = 0.7;
    starting_height = driver.get_stand_height();
    for(int i=0; i<num_stand_height_cycles; i++){
        current_height = starting_height + (static_cast<float>(i)/num_stand_height_cycles) * (target_height-starting_height);
        driver.set_stand_height(current_height);
        std::cout << "    Height: "<<current_height<<" m"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }
    std::cout << "    Done." << std::endl;

    // Move arms up
    std::cout << "Press ENTER to raise arms ...";
    std::cin.get();
    target_joint_pos << 0.f,                      // waist
                        0.f,  1.571,  1.571, 0.f, // left arm
                        0.f, -1.571, -1.571, 0.f; // right arm
    starting_joint_pos = driver.get_all_upper_body_joint_positions();
    for(int i=0; i<num_movement_cycles; i++){
        current_joint_pos = starting_joint_pos + (static_cast<float>(i)/num_movement_cycles)*(target_joint_pos-starting_joint_pos);
        driver.set_all_upper_body_joint_positions(current_joint_pos);
        std::cout << "    Joint angles: "<<current_joint_pos.transpose()<<" (rad)"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }
    std::cout << "    Done." << std::endl;


    // Move torso
    std::cout << "Press ENTER to walk forwards ...";
    std::cin.get();
    desired_torso_velocity<<0.2, 0, 0;
    for(int i=0; i<num_movement_cycles; i++){
        driver.set_torso_velocity(desired_torso_velocity);
        std::cout << "    Torso velocities: "<<desired_torso_velocity.transpose()<<" (m, m, radps)"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }
    driver.set_torso_velocity(VectorXd::Zero(3));
    std::cout << "    Done." << std::endl;

    // Walk and move arms simultaneously
    std::cout << "Press ENTER to move backwards and lower arms ...";
    std::cin.get();
    target_joint_pos << 0.f,                  // waist
                        0.f,  0.f, 0.f, 0.f,  // left arm
                        0.f,  0.f, 0.f, 0.f;  // right arm
    starting_joint_pos = driver.get_all_upper_body_joint_positions();
    desired_torso_velocity<<-0.2, 0, 0;
    std::cout << "    Begin motion..."<<std::endl;
    for(int i=0; i<num_movement_cycles; i++){
        current_joint_pos = starting_joint_pos + (static_cast<float>(i)/num_movement_cycles)*(target_joint_pos-starting_joint_pos);
        driver.set_all_upper_body_joint_positions(current_joint_pos);
        driver.set_torso_velocity(desired_torso_velocity);
        std::cout << "\n    Joint angles: "<<current_joint_pos.transpose()<<" (rad)"<<std::endl;
        std::cout << "    Torso velocities: "<<desired_torso_velocity.transpose()<<" (m, m, radps)"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }
    driver.set_torso_velocity(VectorXd::Zero(3));
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