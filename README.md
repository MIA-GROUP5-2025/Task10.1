# TurtleBot3 Project

## Introduction
This project aims to enhance a TurtleBot3 by deploying its simulation on Gazebo, controlling it via keyboard, reading and visualizing IMU and LiDAR data, converting IMU readings, adding noise and implementing a Kalman filter.

## Requirements
- ROS Noetic
- TurtleBot3 Packages
- Gazebo
- RVIZ
- rqt_multiplot
- rqt

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
### 4. Add gaussian noise to the IMU data
you will need to modify your turtlebot3's associated .xarco file to include noise
1. **Navigate to the file containing the IMU sensor definition**
   ```bash
   cd ~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf
2. **Modify the file with any tool you have**
   ```bash
   gedit turtlebot3_burger.gazebo.xacro
   nano turtlebot3_burger.gazebo.xacro
3. **Add noise to the IMU sensor**
   ```xml
   <gazebo>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <alwaysOn>true</alwaysOn>
    <bodyName>imu_link</bodyName>
    <frameName>imu_link</frameName>
    <topicName>imu</topicName>
    <serviceName>imu_service</serviceName>
    <gaussianNoise>0.5</gaussianNoise>
    <updateRate>1000</updateRate> 
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2.0</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2.0</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2.0</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2.0</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2.0</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2.0</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </plugin>
</gazebo>

4. **Rebuild the workspace and relaunch the node**
   ```bash
   cd ~/catkin_ws
   catkin_make
   rosrun imu_converter imu_converter.py
   rostopic echo /imu_yaw_deg
You will find that the yaw angle is changing while the turtlebot is not moving because of the noise that we added
Before noise:
![imu no noise](https://github.com/user-attachments/assets/8794e3d2-2d27-48be-85a5-09cbedc746e9)

After noise:
![afternoise](https://github.com/user-attachments/assets/704abb31-5a9f-4854-b0c6-e2cd9949e304)

### 5. Implement 1D-Kalman Filter on YAW angle from the IMU
1. **Create a node for the filter**
   ```bash
   cd ~/catkin_ws/src/imu_converter/scripts/
   touch yaw_kalman_filter.py
   chmod +x yaw_kalman_filter.py
2. **Run imu_converter node**
Before running the kalman filter node you have to run the imu converter first, because the kalman filter subscribes on the topic /imu_yaw_deg which the imu converter creates
   ```bash
   rosrun imu_converter imu_converter.py
3. **Run yaw_kalman_filter node**
   ```bash
   rosrun imu_converter yaw_kalman_filter.py
4. **Test the filter**
   ```bash
   rostopic echo /filtered_yaw
After implementing kalman filter:
![afterkalmanfilter](https://github.com/user-attachments/assets/7cc769c5-8c62-47f9-9f56-476fc6552702)

### 6. Visualize the filtered data
1. **To install `rqt`:**
   * Open terminal.
   * Update your package list to ensure you have the latest information and install rqt using the following command::
   ```bash
   sudo apt update
   sudo apt install ros-noetic-rqt
2. **Run** `rqt` by simply typing `rqt` in your terminal.
   >**Problem:** When running `rqt_multiplot`, the window that opened wasn't responding.
3. **Load the** `rqt_multiplot` **plugin:**
    * Go to `Plugins` > `Visualization` > `Multiplot`.
4.  **Click on the ⚙️ button to choose a configuration.**
5. **Add a new plot:**
    * Choose Title
    * Select the topic `/imu_yaw_deg` for the y-axis.
    * Select `data` for the field.
    * For the x-axis, tick `Message Receipt Time`
    * Select `Okay`.
6. **Add new plot: **
    * Repeat the process.
    * Select the topic `/filtered_yaw` for the y-axis.
7. **Run the graph**
    * Select the ▶️ button.

### Saving and Loading Configurations
* **Save configuration:** Click on `Save Configuration`.
* **Load configuration:** Click on `Open Configuration`.

   
