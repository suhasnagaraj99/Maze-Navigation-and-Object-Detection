ENPM 809Y : RWA3 Submission Group 5

This package contains the code for RWA 3 ( A Maze Navigating Robot)

Before running the file make sure all the dependencies are installed as mentioned in the RWA3 Project Statement pdf.


Follow these steps to run the package 
1. Download the package and unzip the contents into the src folder of a workspace containing the following pakages : mage_msgs, ros2_aruco, ros2_aruco_interfaces.
2. Colcon build the packages and make sure the turtle bot model is set to waffle.
3. Launch turltlebot3_gazebo in maze.launch.py using the following command : ros2 launch turtlebot3_gazebo maze.launch.py.
4. Launch the file tbot_nodes.launch.py to start the nodes : aruco_broadcaster, battery_broadcaster and battery_listener using the following command: ros2 launch group5 tbot_nodes.launch.py 
5. To start the maze navigation bot use the following command : ros2 launch group5 tbot_pub.launch.py
6. Wait till the robot fully navigates and stops at the end aruco marker.
7. After the bot stops, switch to the terminal where tbot_nodes.launch.py was launched to obtain information regarding the parts that have been detected by the advanced logical camera.

Note: 
1. Source the workspace before running any executable from the package
2. If the ideal navigation is not observed, close everything and follow steps 3 to 7 again. Due to gazebo issues ideal results are not always obtained.
3. Please ignore the initial error messages displayed when the tbot_nodes.launch.py is launched. This is because sometimes there is a delay with the battery_broadcaster node creation.
