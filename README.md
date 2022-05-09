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
colcon build
source install/local_setup.bash
```

# Usage

* To view the timestamp of camera and lidar.

```bash
ros2 run sensors_pkg sensor_subscriber
```
