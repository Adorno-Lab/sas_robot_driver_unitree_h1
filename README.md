# sas_robot_driver_unitree_h1

![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_robot_driver_unitree_z1)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)

This repository contains a SAS driver and simulation capabilities for the Unitree H1, developed based on the Unitree B1 implementation by Juan José Quiroz Omaña. The H1 drivers are maintained by Daniel S. J. Derwent. For questions or issues, please contact [daniel.derwent@manchester.ac.uk](mailto:daniel.derwent@manchester.ac.uk).


## How to use the simulator
To use the simulator, first ensure that Docker is installed on the host machine, and then clone this repository together with its submodules:

```shell
git clone git@github.com:Adorno-Lab/sas_robot_driver_unitree_h1.git --recursive
```

The simulator image can then be built with:

```shell
sh h1_simulator_build.sh
```

Once the image is built, it can be started with:

```shell
sh h1_simulator_start.sh
```

The simulator uses the [sas_robot_driver_coppeliasim package](https://github.com/MarinhoLab/sas_robot_driver_coppeliasim), which allows the simulated robot to be controlled through a standard SAS interface.

At the time of writing, the simulator is limited to scenarios in which the robot base is fixed and only the upper-body joints are actuated. More generalised capabilities are planned for future releases.

## How to use the hardware drivers

To use the drivers on the real robot, first ensure that Docker is installed on the host computer and set up the H1 on a crane without powering it on yet. The robot is not currently connected to the internet, so all required files must be transferred from the host machine.

Begin by pulling the files needed for transfer to the robot, including this repository:

```shell
git clone git@github.com:Adorno-Lab/sas_robot_driver_unitree_h1.git --recursive
```

You will also need a local copy of the base Docker image, since the robot will be unable to download it in the usual way. To obtain it, run:

```shell
docker pull murilomarinho/sas:jazzy
docker save -o sas.tar murilomarinho/sas:jazzy
```

Once all required files are available, the robot can be powered on and the transfer process can begin.

> [!NOTE]
> Ensure that the Ethernet cable is connected before the robot is powered on, since the arms will obstruct access once the startup procedure begins.

The robot should be suspended above the floor with its arms hanging down, the waist pointing forward, and the ankle and shoulder-roll joints locked at their maximum positions. Activate power, then use `L2+B` on the remote control to enter damping mode. From there, use `L2+UP` to enter preparation mode, which will cause the robot to move.

Once the robot is in preparation mode and the Ethernet connection is available, the required files can be transferred to the onboard computer using:

```shell
scp sas.tar unitree@192.168.123.162:~
scp -r sas_robot_driver_unitree_h1/ unitree@192.168.123.162:~
```

This will prompt for a password; please contact Daniel if needed.

Once the files are in place, connect to the robot via SSH:

```shell
ssh -X unitree@192.168.123.162
```

and begin the process of installing them. Start with the Docker image:

```shell
docker load -i sas.tar
```

Then build the container:

```shell
cd sas_robot_driver_unitree_h1
sh h1_driver_build.sh
```

Once the container has been built, it can be entered with:

```shell
sh h1_driver_start.sh
```

The code can then be built with the alias:

```shell
build_and_source
```

The test executables can be run using:

```shell
build/sas_robot_driver_unitree_h1/<executable_name>
```

For example, `build/sas_robot_driver_unitree_h1/basic_test` runs the example in `basic_test.cpp`. The scripts in `src/standalone_scripts` use the standalone C++ driver class without SAS or ROS functionality to test basic robot features. By contrast, the scripts in `src/ROS2_scripts` use the full SAS/ROS 2 driver interface and require the driver process to already be running in another terminal.

To launch the drivers, use:

```shell
launch_h1_driver <robot>
```

where `<robot>` is `yellow` or `blue` for the real robots, or `dummy` for the dummy robot. The dummy robot allows the drivers to be tested without a connected hardware robot and disables the checks that verify real robot connectivity and communication. 
