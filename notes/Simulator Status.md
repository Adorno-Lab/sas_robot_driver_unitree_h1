# Simulator Status

## 28/09 - Topics

The relevant simulator topics are currently:

```shell
/sas_robot_driver_coppeliasim/h1/get/home_states
/sas_robot_driver_coppeliasim/h1/get/joint_positions_max
/sas_robot_driver_coppeliasim/h1/get/joint_positions_min
/sas_robot_driver_coppeliasim/h1/get/joint_states

/sas_robot_driver_coppeliasim/h1/set/clear_positions
/sas_robot_driver_coppeliasim/h1/set/homing_signal
/sas_robot_driver_coppeliasim/h1/set/shutdown
/sas_robot_driver_coppeliasim/h1/set/target_joint_forces
/sas_robot_driver_coppeliasim/h1/set/target_joint_positions
/sas_robot_driver_coppeliasim/h1/set/target_joint_velocities
/sas_robot_driver_coppeliasim/h1/set/watchdog_trigger
```

The simulator should, ideally, mimic the real driver's topics exactly. These are:

```shell
/sas_h1/Dummy/get/IMU_state
/sas_h1/Dummy/get/home_states
/sas_h1/Dummy/get/imu_orientation
/sas_h1/Dummy/get/joint_positions_max
/sas_h1/Dummy/get/joint_positions_min
/sas_h1/Dummy/get/joint_states
/sas_h1/Dummy/get/stand_height_percent
/sas_h1/Dummy/get/temperatures

/sas_h1/Dummy/set/clear_positions
/sas_h1/Dummy/set/control_mode
/sas_h1/Dummy/set/homing_signal
/sas_h1/Dummy/set/shutdown
/sas_h1/Dummy/set/stand_height_percent
/sas_h1/Dummy/set/target_joint_forces
/sas_h1/Dummy/set/target_joint_positions
/sas_h1/Dummy/set/target_joint_velocities
/sas_h1/Dummy/set/target_twist
```

To make this happen, the following changes are planned:

- Change the robot model name in the simulator so that it follows the same naming convention as the real driver launch files (i.e., `sas_h1/P_Body`, `sas_h1/Atlas` and `sas_h1/Dummy`). Likely new name: `Simulator`.
- Change the topic prefix so that it reads `sas_h1` rather than `sas_robot_driver_coppeliasim`
- Add the below list of topics:

```shell
/sas_h1/Dummy/get/IMU_state
/sas_h1/Dummy/get/imu_orientation
/sas_h1/Dummy/get/stand_height_percent
/sas_h1/Dummy/get/temperatures

/sas_h1/Dummy/set/control_mode
/sas_h1/Dummy/set/stand_height_percent
/sas_h1/Dummy/set/target_twist
```

## 28/09 - Alternative plan

A better plan may be to have the Dummy driver publish information required by the simulator. That way, the simulator's implementation can be relatively minimal, and all important changes happen in the driver itself. 

This would solve some problems with the current plan. For example, suppose that the joint speed limit needed to be adjusted. In the current plan, this would need to be changed twice - once in the driver code and again in the simulator code. In the new plan, this only needs to be changed in the driver code. This idea is worth further consideration.