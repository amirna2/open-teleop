FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-audio-common \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Create ROS workspace
WORKDIR /ros2_ws
COPY . .

# Build ROS packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Set up entry point
COPY ./launch/all_bridges.launch.py /ros2_ws/launch/
RUN chmod +x /ros2_ws/launch/all_bridges.launch.py

# Source ROS environment and launch nodes
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && \
                    source /ros2_ws/install/setup.bash && \
                    ros2 launch all_bridges.launch.py"] 