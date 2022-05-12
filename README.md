# Sensor Package

# Setup environment

* Get the code

```bash
mkdir -p sensors_pkg_ws/src
cd sensors_pkg_ws/src
git clone https://github.com/Adlink-ROS/sensors_pkg.git
cd ../
```

* Build

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

# Usage

* To view the timestamp of camera and lidar.

```bash
ros2 run sensors_pkg sensor_subscriber
```

* Use message filter to get data from camera and lidar.

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
