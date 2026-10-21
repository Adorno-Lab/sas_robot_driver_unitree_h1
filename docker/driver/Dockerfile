# syntax=docker/dockerfile:1

# ####################################################
#                   Basic Setup                      #
# ####################################################

# Start from the base image created by Murillo.
FROM murilomarinho/sas:jazzy

# Use bash for Dockerfile RUN instructions.
SHELL ["/bin/bash", "-c"]

# Default ROS domain; optionally override at runtime with -e ROS_DOMAIN_ID=<value>.
ENV ROS_DOMAIN_ID=42

# Set the workspace as the working directory.
WORKDIR /sas_robot_driver_unitree_h1

# Copy the repository contents into the container.
COPY . /sas_robot_driver_unitree_h1

# ####################################################
#                 Install UnitreeSDK                 #
# ####################################################

RUN cmake -S src/unitree_sdk2 \
          -B /tmp/unitree_sdk2_build \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/unitree_sdk2_build -j$(nproc) && \
    cmake --install /tmp/unitree_sdk2_build

# ####################################################
#        Install the "lego driver" components        #
# ####################################################

RUN cmake -S src/unitree_drivers \
          -B /tmp/unitree_build \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/unitree_build -j$(nproc) && \
    cmake --install /tmp/unitree_build

# ####################################################
#              Build SAS legged messages             #
# ####################################################

# Install outside the driver workspace so that cleaning the
# driver's build/install directories does not delete it.
RUN source /opt/ros/jazzy/setup.bash && \
    colcon --log-base /tmp/sas_legged_msgs_log build \
      --base-paths /sas_robot_driver_unitree_h1/src/sas_tools/sas_legged_msgs \
      --packages-select sas_legged_msgs \
      --build-base /tmp/sas_legged_msgs_build \
      --install-base /opt/sas_legged_msgs

RUN ldconfig

# ####################################################
#                Build the H1 workspace              #
# ####################################################

# Use the same build sequence as the old build_and_source helper.
RUN source /opt/ros/jazzy/setup.bash && \
    source /opt/sas_legged_msgs/setup.bash && \
    rm -rf build install log && \
    colcon build

# ####################################################
#             Interactive terminal helpers           #
# ####################################################

RUN cat <<'EOF' >> /root/.bashrc

# Make ROS, the message interfaces and the driver available.
source /opt/ros/jazzy/setup.bash
source /opt/sas_legged_msgs/setup.bash

if [ -f /sas_robot_driver_unitree_h1/install/local_setup.bash ]; then
    source /sas_robot_driver_unitree_h1/install/local_setup.bash
fi

# Helper retained for development use
build_and_source() {
    (
        # Keep directory and build-environment changes local.
        cd /sas_robot_driver_unitree_h1 || exit 1

        source /opt/ros/jazzy/setup.bash || exit 1
        source /opt/sas_legged_msgs/setup.bash || exit 1

        # Clean only the driver workspace.
        rm -rf build install log

        colcon build
    ) || return 1

    # Make the rebuilt driver available in this terminal.
    source /sas_robot_driver_unitree_h1/install/setup.bash
}

# Launch the real-robot launch file.
launch_h1_driver() {
    if [ "$#" -lt 1 ]; then
        echo "Usage: launch_h1_driver <robot_name> [launch arguments...]"
        return 1
    fi

    local robot_name="$1"
    shift

    ros2 launch sas_robot_driver_unitree_h1 \
        real_H1_launch.py \
        "robot_name:=${robot_name}" \
        "$@"
}
EOF

# ####################################################
#                 Container startup                  #
# ####################################################

# Keep the entrypoint outside the workspace so a development
# bind mount over the workspace wouldn't hide it.
COPY docker/driver/h1_entrypoint.sh /usr/local/bin/h1_entrypoint.sh
RUN chmod +x /usr/local/bin/h1_entrypoint.sh

ENTRYPOINT ["/usr/local/bin/h1_entrypoint.sh"]

# Clear any default command inherited from the base image.
# A robot name must be supplied explicitly.
CMD []
