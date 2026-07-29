#!/bin/bash

# Allow the container to access the host's X server
xhost +local:root

# Run the container with the following arguments:
#
# -it --------------------------------------------> Open the container in the terminal as an interactive session.
# --name sas_robot_driver_unitree_h1 --------------> Assigns the name 'sas_robot_driver_unitree_h1' to the container.
# --rm -------------------------------------------> Automatically removes the container when it exits.
# --privileged -----------------------------------> Grants the container elevated privileges, allowing direct access
#                                                   to hardware such as USB devices.
# --network host ---------------------------------> Shares the host's network namespace with the container.
# --ipc host -------------------------------------> Shares the host's IPC namespace with the container.
# -e DISPLAY=$DISPLAY ----------------------------> Makes the host X display available inside the container.
# -v /tmp/.X11-unix:/tmp/.X11-unix ---------------> Mounts the X11 socket so GUI applications can display windows.
# -v "$(pwd):/sas_robot_driver_unitree_h1" --------> Mounts the current host directory into the container workspace.
# sas_robot_driver_unitree_h1 ---------------------> Tells the container which image to use.

docker run \
  -it \
  --name sas_robot_driver_unitree_h1 \
  --rm \
  --privileged \
  --network host \
  --ipc host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd):/sas_robot_driver_unitree_h1" \
  sas_robot_driver_unitree_h1