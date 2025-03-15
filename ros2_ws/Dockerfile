FROM ros:jazzy

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    libzmq3-dev \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-audio-common \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir pyzmq

# Create ROS workspace
WORKDIR /ros2_ws
COPY . .

# Build ROS packages
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Set up entry point
COPY ./launch/all_bridges.launch.py /ros2_ws/launch/
RUN chmod +x /ros2_ws/launch/all_bridges.launch.py

# Source ROS environment and launch nodes
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && \
                    source /ros2_ws/install/setup.bash && \
                    ros2 launch all_bridges.launch.py"] 