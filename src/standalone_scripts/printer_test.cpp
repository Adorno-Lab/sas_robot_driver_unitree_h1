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

#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>
#include <array>
#include <string>

#include <Eigen/Core>
#include <dqrobotics/DQ.h>

#include <marinholab/sas/core/sas_shutdown_signaler.hpp>

#include <unitree_drivers/DriverUnitreeLowState.h>

#include "DriverUnitreeH1.hpp"

using namespace Eigen;
using namespace DQ_robotics;

// This script repeatedly prints the robot telemetry stream, including joint positions,
// velocities, torques, temperatures, and IMU readings, to validate the low-level driver state outputs.

bool dummy_mode = false; // Set to true to use the dummy driver rather than the real driver

template <std::size_t N>
void print_array(const std::string& array_name,
                 const std::array<double, N>& array_to_print)
{
    std::cout << array_name << ": [ ";

    for (double value : array_to_print)
    {
        std::cout << value << " ";
    }

    std::cout << "]" << std::endl;
}

int main()
{
    // Create robot driver
    std::cout << "Creating robot driver..." << std::endl;
    static std::shared_ptr<marinholab::sas::core::ShutdownSignaler> shutdown_signaler = std::make_shared<marinholab::sas::core::ShutdownSignaler>();
    DriverUnitreeH1 driver("eth0", "position_controlled", false, dummy_mode, shutdown_signaler);
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
    std::cout << "Press ENTER to begin  ...";
    std::cin.get();

    std::atomic<bool> running = true;
    std::thread t([&](){
        while (running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            auto positions = driver.get_upper_body_joint_positions();
            auto velocities = driver.get_upper_body_joint_velocities();
            auto torques = driver.get_upper_body_joint_torques();
            auto joint_temps = driver.get_joint_temperatures();
            // auto battery_soc = driver.get_battery_state_of_charge();
            // auto battery_temps = driver.get_battery_temperatures();
            auto IMU_data = driver.get_IMU_data();

            std::cout<<"positions: "<<positions.transpose()<<std::endl;
            std::cout<<"velocities: "<<velocities.transpose()<<std::endl;
            std::cout<<"torques: "<<torques.transpose()<<std::endl;
            std::cout<<"joint temperatures: "<<joint_temps.transpose()<<std::endl;
            // std::cout<<"battery charge: "<<battery_soc<<std::endl;
            // std::cout<<"casing temperatures: "<<battery_temps.transpose()<<std::endl;

            print_array("IMU quat", IMU_data.quaternion);
            print_array("gyro", IMU_data.gyroscope);
            print_array("accelerometer", IMU_data.accelerometer);
            print_array("Euler angles", IMU_data.rpy);
            std::cout<<"---------------------------------------------------------"<<std::endl;
        }
    });

    std::cout << "Press ENTER to stop...";
    std::cin.get();

    running = false;
    t.join();

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