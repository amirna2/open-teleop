# Setup TurtleBot 4 for ROS Jazzy

### Update System
```
sudo apt update && sudo apt install wget curl gnupg lsb-release
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
```

### Install Gazebo Harmonic and TurtleBot 4 Simulator
```
sudo apt update
sudo apt install gz-harmonic ros-jazzy-turtlebot4-simulator
```

### Launch the Simulation
```
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```
