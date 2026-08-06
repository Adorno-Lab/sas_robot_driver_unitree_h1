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
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("sigterm_timeout", default_value="30"),
            Node(
                package="sas_robot_driver_unitree_h1",
                executable="sas_robot_driver_unitree_h1_node",
                name="P_Body",
                namespace="sas_h1",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "robot_name": "P_Body",
                        "thread_sampling_time_sec": 0.002,
                        "mode": "PositionControl",
                        "ENTER_DAMPING_MODE_ON_DEINIT": False,
                        # "ROBOT_IP": "192.168.8.226", #192.168.123.220
                        # "ROBOT_PORT": 8082,
                    }
                ],
            ),
        ]
    )
