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

"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.

If the variable enable_camera is set to true, then the camera feed will also be published on the topic
/sas_h1/<robot_name>/get/camera_feed. If not, then the camera node will not be launched.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = "sas_robot_driver_unitree_h1"
    robot_name = "P_Body"
    namespace = "sas_h1"
    network_interface = "eth0"
    output = "screen"

    arguments = [
        DeclareLaunchArgument("sigterm_timeout", default_value="30"),
        DeclareLaunchArgument("enable_camera", default_value="false"),
    ]

    # Start the main driver as in the other launch files.
    driver = Node(
        package=package,
        executable="sas_robot_driver_unitree_h1_node",
        name=robot_name,
        namespace=namespace,
        output=output,
        emulate_tty=True,
        parameters=[
            {
                "robot_name": robot_name,
                "thread_sampling_time_sec": 0.002,
                "mode": "position_controlled",
                "ENTER_DAMPING_MODE_ON_DEINIT": False,
                "network_interface": network_interface,
                "DUMMY_MODE": False,
                # "ROBOT_IP": "192.168.8.226", #192.168.123.220
                # "ROBOT_PORT": 8082,
            }
        ],
    )

    # If enable_camera is true, then also start the camera process in its own node.
    camera = Node(
        package=package,
        executable="sas_robot_driver_unitree_h1_camera_node",
        name="robot_name_" + "camera",
        namespace=namespace,
        output=output,
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        parameters=[
            {
                "robot_name": robot_name,
                "network_interface": network_interface,
                "publish_rate": 20.0,  # in Hz
                "scale": 0.5,  # Downscaling of images, 0 to 1
                "jpeg_quality": 70,  # Jpeg quality, 0 to 100.
            }
        ],
    )

    return LaunchDescription(arguments + [driver, camera])
