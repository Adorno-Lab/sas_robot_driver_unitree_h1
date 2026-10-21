# Note: This dockerfile is designed to work without an internet connection, since the H1's are
#       not currently connected to the network. Instead, the files in this repo are moved to the
#       robot via SCP over an ethernet cable, and then the build process occurs. See the Readme 
#       for more information.

# ####################################################
#                   Basic Setup                      #
# ####################################################

# Start from the base image created by Murillo
FROM murilomarinho/sas:jazzy

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Make the message package available in interactive terminals.
# Only source the driver overlay if it has already been built.
RUN cat <<'EOF' >> /root/.bashrc

# SAS legged message interfaces.
source /opt/sas_legged_msgs/setup.bash

# H1 driver overlay, when available.
if [ -f /sas_robot_driver_unitree_h1/install/local_setup.bash ]; then
    source /sas_robot_driver_unitree_h1/install/local_setup.bash
fi
EOF

# Set the workspace as the working directory
WORKDIR /sas_robot_driver_unitree_h1

# Copy the repository contents into the container
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

# # ####################################################
# #              Build SAS legged messages             #
# # ####################################################

# Build the message package explicitly from the submodule.
# Install outside the driver workspace so that cleaning the
# driver's build/install directories does not delete it.
#
# This step uses local sources only. All dependencies must
# already be available in the base image.
RUN source /opt/ros/jazzy/setup.bash && \
    colcon --log-base /tmp/sas_legged_msgs_log build \
      --base-paths /sas_robot_driver_unitree_h1/src/sas_tools/sas_legged_msgs \
      --packages-select sas_legged_msgs \
      --build-base /tmp/sas_legged_msgs_build \
      --install-base /opt/sas_legged_msgs

RUN ldconfig

# ####################################################
#                   Set up Aliases                   #
# ####################################################

# ALIASES
RUN cat <<'EOF' >> ~/.bashrc
build_and_source() {
    (
        # Run the build in a subshell so directory changes and build-environment changes remain local.
        cd /sas_robot_driver_unitree_h1 || exit 1

        # Ensure the prebuilt message package is available.
        source /opt/sas_legged_msgs/setup.bash || exit 1

        # Clean only the driver workspace.
        rm -rf build install log

        colcon build
    ) || return 1

    # Make the rebuilt driver available in this terminal.
    source /sas_robot_driver_unitree_h1/install/setup.bash
}

launch_h1_driver() {
    if [ $# -ne 1 ]; then
        echo "Usage: launch_driver <launch_file>"
        return 1
    fi

    ros2 launch sas_robot_driver_unitree_h1 $1_H1_launch.py
}

EOF

# ####################################################
#                 End of Dockerfile                  #
# ####################################################
