# TurtleBot3 Project

## Task 10.1
## Introduction
This project aims to enhance a TurtleBot3 by deploying its simulation on Gazebo, controlling it via keyboard, reading and visualizing IMU and LiDAR data, converting IMU readings, adding noise and implementing a Kalman filter.

## Requirements
- ROS Noetic
- TurtleBot3 Packages
- Gazebo
- RVIZ
- rqt_multiplot

## Steps

### 1. Deploy TurtleBot3 Simulation on Gazebo
1. **Install ROS and TurtleBot3 Packages**:
   ```bash
   sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
2. **Set Up Your Workspace**
   ```bash
   mkdir -p ~/turtlebot3_ws/src
   cd ~/turtlebot3_ws/
   catkin_make
   source devel/setup.bash
3. **Launch Gazebo Simulation**
   ```bash
   export TURTLEBOT3_MODEL=burger
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
4. **Control TurtleBot3 with Keyboard**
   ```bash
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
### 2. Read IMU Data and Visualise LiDAR Data
1. **Launch RVIZ**
   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
2. **Read IMU Data**
   ```bash
   rostopic echo /imu
3. **Visualize LiDAR Data**
Add a LaserScan display in RVIZ and set the topic to /scan.
    * Add a `LaserScan` display in RVIZ and set the topic to `/scan`.
### 3. Convert IMU Readings from Quaternion to Degree and Publish on a New Topic
1. **Create a ROS package**
   ```bash
   cd ~/catkin_ws/src
   catkin_create_pkg imu_converter rospy std_msgs sensor_msgs
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
2. **Write the Conversion Node**
   * Create a Python script to convert quaternion to Euler angles and publish the yaw angle in degrees.
   * Create a new file `imu_converter.py` in the `scripts` directory of your package:
        ```bash
        cd ~/catkin_ws/src/imu_converter/scripts
        touch imu_converter.py
        chmod +x imu_converter.py
3. **Update the Package Configuration:**
   Edit `CMakeLists.txt` to add the following lines:
    ```bash
    catkin_install_python(PROGRAMS scripts/imu_converter.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
4. **Build the workspace**
5. **Run the node**
    ```bash
    rosrun imu_converter imu_converter.py
6. **Verify IMU data is being published**
    ```bash
    rostopic echo /imu_yaw_deg
