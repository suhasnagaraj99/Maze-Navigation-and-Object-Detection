# Maze-Navigation-and-Object-Detection

This repository contains the submission for ENPM809Y RWA 3  - A ROS package to move a turtlebot in a maze. To move through the maze, the turtlebot relies on Aruco markers. While the turtlebot navigates the maze, it will need to ﬁnd and report objects found in the environment.

![Video GIF](https://github.com/suhasnagaraj99/Maze-Navigation-and-Object-Detection/blob/main/rwa3.gif)

## Repository Structure
- **group5**: Package for autonomous maze navigation and object detection.
- **mage_msgs**: Package for defining custom ros messages
- **ros2_aruco**: Package for reading aruco markers
- **turtlebot3_sensors**: Package containing turtlebot3 camera plugin scripts
- **turtlebot3_simulations**: Package for simulating turtlebot3 on gazebo

## Prerequisites
- Before running the code, ensure that you have the following installed:
  - ROS2 (recommended version: Humble)
  - Gazebo
  - OpenCV
  - OpenCV Contrib
  - numpy (version < 2)

## Setup Instructions

1. **Run the following lines of code to set up the environment**
   ```bash
   sudo apt update
   sudo apt install python3-pip
   pip3 uninstall opencv-python opencv-contrib-python
   pip3 install opencv-python opencv-contrib-python
   pip3 install "numpy<2"
   echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
   source ~/.bashrc
   ```
2. **Create a Workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
3. **Copy Packages**:
   - Paste the packages, which are inside the zip folder `suhas99_enpm809y_rwa3.zip`, into the src folder of your workspace. Also paste the package `group5` in the same src folder. 
     
4. **Build and Source the Packages**:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
   
## Running the Simulation

1. **Launch Turtlebot3 Waffle in Gazebo with Custom World**:
   ```bash
   ros2 launch turtlebot3_gazebo maze.launch.py
   ```
   
2. **Launch the file `tbot_nodes.launch.py` to start the nodes : `aruco_broadcaster`, `battery_broadcaster` and `battery_listener`**:
   ```bash
   ros2 launch group5 tbot_nodes.launch.py 
   ```
   
3. **Launch the file `tbot_pub.launch.py` to start maze navigation**:
   ```bash
   ros2 launch group5 tbot_pub.launch.py
   ```
     
4. **Results**:
   - Wait till the robot fully navigates and stops at the end aruco marker
   - After the bot stops, switch to the terminal where tbot_nodes.launch.py was launched to obtain information regarding the parts that have been detected by the advanced logical camera.

![alt text](https://github.com/suhasnagaraj99/Maze-Navigation-and-Object-Detection/blob/main/rwa3.png?raw=false)

## Acknowledgments

This project utilizes code and resources from the [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) repository. We would like to thank the TurtleBot3 team for their contributions and the open-source community for providing these valuable resources.
