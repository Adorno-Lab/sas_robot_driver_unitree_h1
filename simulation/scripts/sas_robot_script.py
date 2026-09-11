# This file was originally distributed with the
# sas_robot_driver_coppeliasim package by Murilo
# M. Marinho
# https://github.com/MarinhoLab/sas_robot_driver_coppeliasim

from sas_robot_driver_coppeliasim import SimROS2Manager, SimROS2RobotManager


def sysCall_init():
    self.sim = require("sim")
    self.sas_manager = SimROS2Manager(self.sim)
    robot_base_handle = self.sim.getObject("..")
    self.sas_manager.sys_call_init()
    sas_robot_manager = SimROS2RobotManager(
        topic_prefix=self.sim.getObjectAlias(robot_base_handle, 1),
        rclcpp_node=self.sas_manager.rclcpp_node,
        sim=self.sim,
        robot_base_handle=robot_base_handle,
    )
    self.sas_manager.node.add_robot_manager(sas_robot_manager)


def sysCall_actuation():
    self.sas_manager.sys_call_actuation()


def sysCall_sensing():
    self.sas_manager.sys_call_sensing()


def sysCall_cleanup():
    self.sas_manager.sys_call_cleanup()
