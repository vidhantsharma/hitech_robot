# Use the official ROS Noetic base image
FROM ros:noetic-ros-base

# Set environment variables
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

# Install necessary dependencies for ROS, RViz, and Gazebo
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    python3-pip \
    python3-rosdep \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-rviz \
    ros-noetic-gazebo-ros-pkgs \
    gazebo11 \
    libgl1-mesa-glx \
    libx11-dev \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Set the working directory
WORKDIR /hitech_robot_ws

# Copy the entire workspace into the container
COPY . /hitech_robot_ws

# Install the workspace dependencies using rosdep
RUN rosdep install --from-paths src --ignore-src -r -y

# Clean build artifacts and build the workspace
RUN rm -rf build/ devel/ && \
    /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"

# Source the setup.bash for the workspace in the .bashrc file
RUN echo "source /hitech_robot_ws/devel/setup.bash" >> ~/.bashrc
