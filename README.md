# sas_robot_driver_unitree_h1

![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_robot_driver_unitree_z1)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)

This repository contains a SAS driver for the Unitree H1, developed based on that for the Unitree B1 by Juan José Quiroz Omaña. The H1 drivers are being maintained by Daniel S. J. Derwent. Feel free to contact me at [daniel.derwent@manchester.ac.uk](mailti:daniel.derwent@manchester.ac.uk) with any questions or issues.

## Instructions

To use this driver, first ensure that you have a computer with docker installed, and set up the H1 on a crane (though don't power it on yet). The robot is not currently connected to the internet, so file transfer must be via your computer. Begin by pulling the files you will need to transfer to the robot, these include this repo:

```shell
git clone git@github.com:Adorno-Lab/sas_robot_driver_unitree_h1.git --recursive
```

and you will also need a local copy of the base Docker image, since the robot will be unable to download this in the usual way. To get that, do:

```shell
docker pull murilomarinho/sas:jazzy
docker save -o sas.tar murilomarinho/sas:jazzy
```

Now you have everything you need, so its time to send them over to the robot. To do that, the robot will need to be powered on.

> [!NOTE]
> Make sure to plug in the ethernet cable before powering on the robot, because the arms will be in the way once this process has begun.

The robot should be suspended above the floor, with its arms straight down, the waist pointing forward, and the ankle and shoulder-roll joints locked at their maximum positions. Active the power, and then use `L2+B` on the remote control to enter damping mode. From there, use `L2+UP` to enter preparation mode (this will cause the robot to move).

Once the robot is in preparation mode, assuming you are connected to the ethernet cable, you can send the relevant files to the onboard computer using:

```shell
scp sas.tar unitree@192.168.123.162:~
scp sas_robot_driver_unitree_h1/ unitree@192.168.123.162:~
```

This will prompt you for a password (ask Daniel if required).

Now the files are where they need to be, connect to the robot via SSH:

```shell
ssh -X unitree@192.168.123.162
```

and begin the process of installing them. Start with the Docker image:

```shell
docker load -i sas.tar
```

Then build the container

```shell
cd sas_robot_driver_unitree_h1
sh dev_build.sh
```

Once the container has been built, you can enter it with

```shell
sh dev_start.sh
```

Then the code can be build with the alias:

```shell
build_and_source
```

The test executables can be run using:

```shell
build/sas_robot_driver_unitree_h1/<executable_name>
```

for example, `build/sas_robot_driver_unitree_h1/basic_test`, will run the example in `basic_test.cpp`. The scripts in `src/standalone_scripts` use the standalone C++ driver class (with no SAS / ROS functionality) in order to test basic features. In contrast, those in `src/ROS2_scripts` use the full SAS / ROS2 driver interface, and the drivers must already be running in another terminal before these scripts can be used.

To launch the drivers use

```shell
launch_h1_driver <robot>
```

where `<robot>` is `yellow` or `blue` (for the real robots) or `dummy` for the dummy robot. The dummy robot allows the drivers to be tested while not connected to a real robot, disabling the checks that ensure that a real robot is connected and communicating. 
