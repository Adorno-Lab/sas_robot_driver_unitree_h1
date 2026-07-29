#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>

#include <Eigen/Core>

#include "DriverUnitreeH1.hpp"

using namespace Eigen;

int main()
{
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
    std::cout << "Press ENTER to begin  ...";
    std::cin.get();

    std::atomic<bool> running = true;
    std::thread t([&](){
        while (running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            auto positions = driver.get_upper_body_joint_positions();
            auto velocities = driver.get_upper_body_joint_velocities();
            std::cout<<"positions: "<<positions.transpose()<<std::endl;
            std::cout<<"velocities: "<<velocities.transpose()<<std::endl;
            std::cout<<"---------------------------------------------------------"<<std::endl;
        }
    });

    std::cout << "Press ENTER to stop...";
    std::cin.get();

    running = false;
    t.join();

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