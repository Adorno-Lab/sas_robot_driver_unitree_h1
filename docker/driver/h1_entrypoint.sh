#!/usr/bin/env bash

# Stop immediately if environment setup or another command fails.
set -e

# Non-interactive startup does not rely on .bashrc, so we have to source some things manually here
source /opt/ros/jazzy/setup.bash
source /opt/sas_legged_msgs/setup.bash
source /sas_robot_driver_unitree_h1/install/local_setup.bash

# Set the ROS domain value.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Handle special cases before interpreting a robot name.
case "${1:-}" in
    bash|/bin/bash)
        # Open a development shell without starting either driver.
        exec "$@"
        ;;

    dummy)
        # Launch the dummy driver and forward any launch arguments.
        shift

        echo "Starting dummy H1 driver (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})."

        exec ros2 launch sas_robot_driver_unitree_h1 \
            dummy_H1_launch.py \
            "$@"
        ;;
esac

# Normal startup requires an explicit robot name.
if [[ "$#" -eq 0 || -z "$1" ]]; then
    echo "Usage:" >&2
    echo "  IMAGE <robot_name> [launch arguments...]" >&2
    echo "  IMAGE dummy [launch arguments...]" >&2
    echo "  IMAGE bash" >&2
    exit 1
fi

robot_name="$1"
shift

echo "Starting H1 driver for '${robot_name}' (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})."

# Replace this shell with ROS launch for container signal handling.
exec ros2 launch sas_robot_driver_unitree_h1 \
    real_H1_launch.py \
    "robot_name:=${robot_name}" \
    "$@"