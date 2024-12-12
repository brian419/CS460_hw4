# Final Project ROS2 Notes

## CS460
Anastasia Spencer

### How to Run

In First terminal in this order:
```

cd f24_robotics
colcon build
source install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch miata_hw4 f23_robotics_1_launch.py

```

In Second Terminal in this order:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run miata_hw4 miata_hw4

```