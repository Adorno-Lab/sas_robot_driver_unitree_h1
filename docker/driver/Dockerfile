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
  
# Add the sourcing of the ROS2 overlay to the bashrc file.
RUN echo "# Source the ROS2 overlay, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc

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

# ####################################################
#                   Set up Aliases                   #
# ####################################################

# ALIASES
RUN cat <<'EOF' >> ~/.bashrc
alias build_and_source='[ -d build ] && rm -rf build; [ -d install ] && rm -rf install; [ -d log ] && rm -rf log; colcon build && source install/setup.bash'

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
