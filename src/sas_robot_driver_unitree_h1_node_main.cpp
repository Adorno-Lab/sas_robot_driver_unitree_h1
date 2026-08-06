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
#   based on the version of this file in the unitree B1 driver by Juan Jose Quiroz Omana
#   https://github.com/Adorno-Lab/sas_robot_driver_unitree_h1/tree/main
#
# ################################################################
*/

#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
//#include <sas_robot_driver_unitree_z1/sas_robot_driver_unitree_z1.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include <sas_robot_driver_unitree_h1/sas_robot_driver_unitree_h1.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler = std::make_shared<sas::ShutdownSignaler>();
void sig_int_handler(int)
{
    shutdown_signaler->shutdown();
}

int main(int argc, char** argv)
{

    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_unitree_h1");

    try
    {
        sas::RobotDriverUnitreeH1Configuration robot_driver_unitree_h1_configuration;
        sas::get_ros_parameter(node,"network_interface", robot_driver_unitree_h1_configuration.network_interface);
        sas::get_ros_parameter(node,"mode", robot_driver_unitree_h1_configuration.mode);
        sas::get_ros_parameter(node,"ENTER_DAMPING_MODE_ON_DEINIT", robot_driver_unitree_h1_configuration.ENTER_DAMPING_MODE_ON_DEINIT);
        sas::get_ros_parameter(node,"robot_name", robot_driver_unitree_h1_configuration.robot_name);
        // sas::get_ros_parameter(node,"ROBOT_IP", robot_driver_unitree_h1_configuration.ROBOT_IP);
        // sas::get_ros_parameter(node,"ROBOT_PORT", robot_driver_unitree_h1_configuration.ROBOT_PORT);

        auto robot_driver_unitree_h1 = std::make_shared<sas::RobotDriverUnitreeH1>(node,
                                                                        robot_driver_unitree_h1_configuration,
                                                                        shutdown_signaler);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();

        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_unitree_h1,
                                             robot_driver_ros_configuration,
                                             shutdown_signaler);
        robot_driver_ros.control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    return 0;


}
