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

    // // The task
    // std::cout << "Press ENTER to attempt invalid position control task ...";
    // std::cin.get();
    
    // std::cout << "    Defining variables..."<<std::endl;
    // float sleep_time_sec = 0.02;
    // float num_cycles = 4.0 / sleep_time_sec;
    // VectorXd target_pos(9);
    // target_pos << 0.0f, 0.f, 0.0f, -1.3,
    //               0.0f, 0.f, 0.0f, 0.f, 
    //               0.f;
    // std::cout << "    Get current joint positions..."<<std::endl;
    // VectorXd starting_pos = driver.get_upper_body_joint_positions();
    // std::cout << "    Begin motion..."<<std::endl;
    // for(int i=0; i<num_cycles; i++){
    //     driver.set_upper_body_joint_positions(starting_pos + (static_cast<float>(i)/num_cycles)*(target_pos-starting_pos));
    //     std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    // }
    // std::cout << "    Done." << std::endl;

    // The task
    std::cout << "Press ENTER to attempt invalid position control task ...";
    std::cin.get();
    
    std::cout << "    Getting ready..."<<std::endl;
    float sleep_time_sec = 0.02;
    float num_cycles = 4.0 / sleep_time_sec;
    VectorXd target_pos(9);
    target_pos << 1.571, 0.f, 0.0f, 0.0f,
                  0.0f, 0.f, 0.0f, 0.f, 
                  0.f;
    VectorXd starting_pos = driver.get_upper_body_joint_positions();
    for(int i=0; i<num_cycles; i++){
        driver.set_upper_body_joint_positions(starting_pos + (static_cast<float>(i)/num_cycles)*(target_pos-starting_pos));
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }
    std::cout << "    Ready." << std::endl;
    std::cout << "Press ENTER to drop arms illegally fast ...";
    std::cin.get();
    
    std::cout << "    Go!"<<std::endl;
    VectorXd next_target_pos(9);
    next_target_pos << 0.0f, 0.f, 0.0f, 0.0f,
                  0.0f, 0.f, 0.0f, 0.f, 
                  0.f;
    for(int i=0; i<num_cycles; i++){
        driver.set_upper_body_joint_positions(next_target_pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_sec / 0.001f)));
    }
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