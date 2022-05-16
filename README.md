# Sensor Package

# Setup environment

* Install necessary packages

```bash
sudo apt install wget
```

* Get the code

```bash
mkdir -p sensors_pkg_ws/src
cd sensors_pkg_ws
wget https://raw.githubusercontent.com/Adlink-ROS/sensors_pkg/main/sensors.repos
vcs import src < sensors.repos
```

* Build xsens library
  - Refer to https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver

```bash
pushd src/bluespace_ai_xsens_ros_mti_driver/lib/xspublic && make && popd
```

* Build

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

# Usage

* To view the timestamp of camera, lidar and imu.

```bash
ros2 run sensors_pkg sensor_subscriber
```

* Use message filter to get data from camera, lidar and imu.

```bash
ros2 run sensors_pkg msg_filter
```

* Launch all the sensors and message filter at the same time

```bash
ros2 launch sensors_pkg launch_sensors.launch.py
```

# Test

To test the program, you need to run sample rosbag

```bash
tar zvxf resource/rosbag2_2022_05_10-02_06_12.tar.gz
ros2 bag play rosbag2_2022_05_10-02_06_12
```
