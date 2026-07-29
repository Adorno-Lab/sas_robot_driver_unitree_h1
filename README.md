![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_robot_driver_unitree_z1)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)


# sas_robot_driver_unitree_h1 - "Clean Slate" branch

> [!CAUTION]
> This repository stores a WIP SAS driver for the Unitree H1. The driver is in very early development. Currently, this code is not suitable for use by anyone.

This branch contains a different approach to the H1 SAS driver development in which, rather than editing the B1 driver to fit the H1, we instead start developing the H1 driver from a "clean slate". Beginning by developing a simple C++ class for interfacing with the robot, which will then be wrapped with a different class that handles the SAS interface. This was the approach taken by Juan to develop the B1 driver.

Currently a minimum viable version of the C++ class has been created, but there remain many features to add. SAS has not yet been integrated.

### Instructions

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

And executables can be run using:
```shell
build/sas_robot_driver_unitree_h1/<executable_name> <arguments>
```

for example, `build/sas_robot_driver_unitree_h1/basic_test eth0`, will run the example in `basic_test.cpp`.

> [!NOTE]
> The network interface to use for executable commands is always `eth0` if the code is running on the robot's onboard PC.