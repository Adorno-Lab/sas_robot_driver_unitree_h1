![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_robot_driver_unitree_z1)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)


# sas_robot_driver_unitree_h1

> [!CAUTION]
> This repository stores a WIP SAS driver for the Unitree H1. The driver is in very early development, having only recently been forked from the B1 driver. Currently, this code is not suitable for use by anyone. What follows is the text from the original README, but it's instructions will likely not work at the moment.

### Docker Instructions

#### Prerequisites:
- Docker with sudo permissions.
- Prepare the Unitree H1 robot.
- Connect your machine to the WiFi H1 network (unitree-h1-5g-xx)

1. Clone this repository
```shell
cd ~/Downloads
git clone https://github.com/Adorno-Lab/sas_robot_driver_unitree_h1 --recursive
cd sas_robot_driver_unitree_h1
```
2. Build the docker image
   
> [!IMPORTANT]
> The argument of the following command sets the ROS_DOMAIN_ID

```shell
sh build_sas_rd_unitree_h1_docker.sh 1
```
3. Start the docker container
```shell
sh start_sas_rd_unitree_h1_docker.sh  
```
4. Build the ROS2 packages
```shell
buildros2
```
5. Start the driver
> [!CAUTION]
> The robot could move with the next command. Be ready to perform an emergency stop!

```shell
launch_ROS2_drivers
```

![start_ROS2_drivers](https://github.com/user-attachments/assets/c30a7867-d738-4423-970a-9a18e476b080)

6. To stop the robot, press `Ctrl + C`

![ctrlc](https://github.com/user-attachments/assets/a206fc3e-5065-406d-9a82-2624c197b9a3)

![h1_stop](https://github.com/user-attachments/assets/e9d26807-dccf-4afe-b802-451619140593)



