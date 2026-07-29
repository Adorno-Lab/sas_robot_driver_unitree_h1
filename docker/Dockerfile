# Start from the base image created by Murillo
FROM murilomarinho/sas:jazzy

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Make sure all the packages are up to date
# RUN apt-get update && apt-get install -y

# Install additional packages
# None yet required
    
# Add the sourcing of the ROS2 overlay to the bashrc file.
RUN echo "# Source the ROS2 overlay, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc

# Set the workspace as the working directory
WORKDIR /sas_robot_driver_unitree_h1

# Copy the repository contents into the container
# In the future, it would be nice if this was a call to clone the repo. This isn't possible currently because 
# the repo is private.
COPY . /sas_robot_driver_unitree_h1

# ALIASES
RUN cat <<'EOF' >> ~/.bashrc
alias build_and_source='[ -d build ] && rm -rf build; [ -d install ] && rm -rf install; [ -d log ] && rm -rf log; colcon build && source install/setup.bash'

EOF
