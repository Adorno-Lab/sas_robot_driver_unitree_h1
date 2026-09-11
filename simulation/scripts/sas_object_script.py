# This file was originally distributed with the
# sas_robot_driver_coppeliasim package by Murilo
# M. Marinho
# https://github.com/MarinhoLab/sas_robot_driver_coppeliasim

from sas_robot_driver_coppeliasim import SimROS2Manager, SimROS2ObjectManager


def sysCall_init():
    self.sim = require("sim")
    self.sas_manager = SimROS2Manager(self.sim)
    self.sas_manager.sys_call_init()
    object_handle = self.sim.getObject("..")
    sas_object_manager = SimROS2ObjectManager(
        object_handle=object_handle, node=self.sas_manager.node, sim=self.sim
    )
    self.sas_manager.node.add_object_manager(sas_object_manager)
    self.sas_manager.sys_call_init()


def sysCall_actuation():
    self.sas_manager.sys_call_actuation()


def sysCall_sensing():
    self.sas_manager.sys_call_sensing()


def sysCall_cleanup():
    self.sas_manager.sys_call_cleanup()
