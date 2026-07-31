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
    VectorXd desired_torques(19);
    desired_torques << 0.f, 0.f, 0.f, 0.f,
                       0.f, 0.f, 0.f, -0.8,
                       0.f;

    std::cout << "    Begin motion..."<<std::endl;
    for(int i=0; i<num_cycles; i++){
        driver.set_upper_body_joint_torques(desired_torques);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    driver.set_upper_body_joint_torques(VectorXd::Zero(19));
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