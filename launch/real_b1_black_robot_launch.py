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
                package="sas_robot_driver_unitree_b1",
                executable="sas_robot_driver_unitree_b1_node",
                name="b1_2",
                namespace="sas_b1",
                # output='screen',
                # emulate_tty=True,
                parameters=[
                    {
                        "robot_name": "b1_2",
                        "thread_sampling_time_sec": 0.002,
                        "mode": "VelocityControl",
                        "LIE_DOWN_ROBOT_WHEN_DEINITIALIZE": False,
                        "ROBOT_IP": "192.168.8.226",  # 192.168.123.220
                        "ROBOT_PORT": 8082,
                        "FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO": True,  # In the black robot, this is required to stop the robot when the velociites are zero.
                    }
                ],
            ),
        ]
    )
