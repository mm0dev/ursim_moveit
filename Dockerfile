FROM osrf/ros:humble-desktop

# 1. Install UR Drivers & MoveIt
RUN apt-get update && apt-get install -y \
    ros-humble-ur-robot-driver \
    ros-humble-ur-moveit-config \
    ros-humble-moveit \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# 2. Source ROS2 automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 3. Create a workspace folder
WORKDIR /root/ros2_ws

CMD ["sleep", "infinity"]