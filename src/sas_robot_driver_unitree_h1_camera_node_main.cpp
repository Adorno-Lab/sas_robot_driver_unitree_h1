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

#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_robot_driver_unitree_h1/sas_robot_driver_unitree_h1_camera.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_unitree_h1_camera");
    int exit_code = 0;

    try
    {
        UnitreeH1CameraConfiguration unitree_h1_camera_configuration;
        sas::get_ros_parameter(node,"robot_name", unitree_h1_camera_configuration.robot_name);
        sas::get_ros_parameter(node,"network_interface", unitree_h1_camera_configuration.network_interface);
        sas::get_ros_parameter(node,"publish_rate", unitree_h1_camera_configuration.publish_rate);
        sas::get_ros_parameter(node,"scale", unitree_h1_camera_configuration.scale);
        sas::get_ros_parameter(node,"jpeg_quality", unitree_h1_camera_configuration.jpeg_quality);

        auto robot_driver_unitree_h1_camera = std::make_shared<UnitreeH1CameraHandler>(node,
                                                                        unitree_h1_camera_configuration);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        rclcpp::spin(node);

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    return 0;

}