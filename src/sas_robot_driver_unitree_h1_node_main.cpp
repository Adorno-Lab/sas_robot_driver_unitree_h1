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
#   Acknowledgement: This file was adapted from the file 
#                    src/sas_robot_driver_unitree_g1_node_main.cpp written by Juan
#                    Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk) for
#                    the unitree G1 driver package:
#                    https://github.com/Adorno-Lab/sas_robot_driver_unitree_g1/tree/main
#
# ################################################################
*/

#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_tools/sas_legged_robot_driver_ros.hpp>
#include <sas_robot_driver_unitree_h1/sas_robot_driver_unitree_h1.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler = std::make_shared<sas::ShutdownSignaler>();
void sig_int_handler(int)
{
    shutdown_signaler->shutdown();
}

/**
 * Parameters (set by the launch file):
 *   - robot_name (string, mandatory):                    Name of the robot, used to determine topic names.
 *   - network_interface (string, mandatory):             e.g. "eth0" on the onboard computer.
 *   - thread_sampling_time_sec (double, mandatory):      period of the control loop, e.g. 0.002.
 *   - twist_timeout_sec (double, optional, default 0.2): a zero twist is sent when no twist arrives in time.
 *   - mode (string, mandatory):                          how the robot is actuated, "position_controlled", 
 *                                                        "velocity_controlled", or "torque_controlled".
 *   - ENTER_DAMPING_MODE_ON_DEINIT (bool, mandatory):    Wether the robot should enter damping mode upon deinitialization
 *   - DUMMY_MODE (bool, mandatory):                      Wether the driver should start in "Dummy" mode (see readme).  
 *
 * The topic prefix is the name of the node (set by the launch file), inside its namespace, e.g. the robot
 * "P_Body" in the namespace "sas_h1" results in sas_h1/P_Body/...
 */

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    // Keep our SIGINT handler: it signals the shutdown so that the control loop ends and the driver
    // deinitializes the robot before ROS is shut down.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_unitree_h1");

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverUnitreeH1Configuration robot_configuration;
        sas::get_ros_parameter(node,"network_interface", robot_configuration.network_interface);
        sas::get_ros_parameter(node,"mode", robot_configuration.mode);
        sas::get_ros_parameter(node,"ENTER_DAMPING_MODE_ON_DEINIT", robot_configuration.ENTER_DAMPING_MODE_ON_DEINIT);
        sas::get_ros_parameter(node,"DUMMY_MODE", robot_configuration.DUMMY_MODE);
        sas::get_ros_parameter(node,"robot_name", robot_configuration.robot_name);

        sas::LeggedRobotDriverROSConfiguration ROS_class_configuration{};
        ROS_class_configuration.robot_driver_ros.robot_driver_provider_prefix = node->get_name();
        sas::get_ros_parameter(node, "thread_sampling_time_sec", ROS_class_configuration.robot_driver_ros.thread_sampling_time_sec);
        sas::get_ros_optional_parameter(node, "twist_timeout_sec", ROS_class_configuration.twist_timeout_sec, 0.2);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK: control mode " << robot_configuration.mode
                                                    << ", network_interface " << robot_configuration.network_interface
                                                    << ", prefix " << node->get_fully_qualified_name()
                                                    << ", thread_sampling_time_sec " << ROS_class_configuration.robot_driver_ros.thread_sampling_time_sec
                                                    << ", twist_timeout_sec " << ROS_class_configuration.twist_timeout_sec);

        auto robot_driver = std::make_shared<sas::RobotDriverUnitreeH1>(robot_configuration, shutdown_signaler);

        sas::LeggedRobotDriverROS legged_robot_driver_ros(node, robot_driver, ROS_class_configuration, shutdown_signaler);
        legged_robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    rclcpp::shutdown();
    return 0;
}