# README #

### Overview of the program ###

This program demonstrates how to use ROS2 build of the Aeva API to publish ROS2 messages from the sensor. Unlike the `ros_aeva_pointcloud_pub` example which triggers callbacks with Aeva API types, this version of the API directly provides the converted ROS2 types.

### Program dependencies ###

AevaAPI
ROS2 (recommended distribution ROS2 Foxy Fitzroy)
cmake (minimum version required 3.10)

### Steps to run the program ###

* Install the provided Aeva ROS2 API Debian package on your system.
```bash
sudo dpkg -i AeriesII_API_<VERSION>_ROS2_<ARCH>.deb
```

* Create a workspace directory with the name `aeva_ws`.
```bash
cd ~/
mkdir aeva_ws
```

* Create a `src` folder inside the workspace.
```bash
cd aeva_ws
mkdir src
```

* Create a ROS2 package inside `src` folder with the package name as `ros2_aeva_publisher`.
```bash
cd src
ros2 pkg create --build-type ament_cmake ros2_aeva_publisher --dependencies rclcpp aeva_msgs geometry_msgs nav_msgs sensor_msgs std_msgs tf2_msgs visualization_msgs
```

* Copy the provided `aeva_msgs` ROS2 package from the ROS2 Aeva API archive to the `aeva_ws/src` directory.



* Copy all source files to `aeva_ws/src/ros2_aeva_publisher/src` directory.

* __IMPORTANT:__ Please note that this example uses the ROS2 build of the Aeva API and not the standard Aeva API.

* __IMPORTANT:__ Replace the auto-generated `CMakeLists.txt` with the provided `CMakeLists.txt` file inside the package directory. The file structure should look like:
```bash
aeva_ws/

aeva_ws/
└── src/
    ├── ros2_aeva_publisher/
    │   ├── include/
    │   │   └── ros2_aeva_publisher/
    │   ├── src/
    │   │   └── ros2_aeva_publisher.cc
    │   ├── CMakeLists.txt (Replace this file with the given CMakeLists.txt)
    │   └── package.xml
    └── aeva_msgs/**/

```

* Go to the workspace directory and run the following command.
```bash
cd ~/aeva_ws
colcon build
```
This builds the ROS2 package with the name `ros2_aeva_publisher`.

* Source the `setup.bash` file from the `install` directory.
```bash
source install/setup.bash
```

* Run the following command from the workspace directory. The program takes the sensor IP address and IDs as the inputs.
* Note that the SENSOR_NAME shouldn't begin with a number, since it's used to derive the message topic. ROS2 topics can't start with a number.
```bash
cd ~/aeva_ws
LD_LIBRARY_PATH=/opt/aeva/aeva-api/ros2/library:$LD_LIBRARY_PATH ros2 run ros2_aeva_publisher ros2_aeva_publisher SENSOR_IP_0 SENSOR_NAME_0 [... SENSOR_IP_N SENSOR_NAME_N]
```
