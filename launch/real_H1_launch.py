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

"""
Launch the real Unitree H1 driver.

WARNING:
    This activates the real robot if the connection is successful.
    Ensure the robot is safe to operate before launching.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Resolve the robot identity from the launch arguments.
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription(
        [
            # No default: the operator must explicitly identify the robot.
            DeclareLaunchArgument(
                "robot_name",
                description="Robot identity, used in the topic and node names.",
            ),
            DeclareLaunchArgument("sigterm_timeout", default_value="30"),
            Node(
                package="sas_robot_driver_unitree_h1",
                executable="sas_robot_driver_unitree_h1_node",
                name=robot_name,
                namespace="sas_h1",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "robot_name": ParameterValue(robot_name, value_type=str),
                        "thread_sampling_time_sec": 0.002,
                        "twist_timeout_sec": 0.2,
                        "mode": "position_controlled",
                        "ENTER_DAMPING_MODE_ON_DEINIT": False,
                        "network_interface": "eth0",
                        "DUMMY_MODE": False,
                        # "ROBOT_IP": "192.168.8.226", #192.168.123.220
                        # "ROBOT_PORT": 8082,
                    }
                ],
            ),
        ]
    )
