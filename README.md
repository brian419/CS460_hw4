Jeongbin Son

### The Miata Algorithm

Our algorithm uses a rudimentary "wall-follow" algorithm.
At the start, the bot moves forward until it finds a wall. Once it finds a wall, it will turn until the right sensor readings from the LIDAR sensor no longer increase, meaning the bot is facing along the wall. Once this occurs, the bot is considered to be following the wall.
From here, the bot moves forward. If the distance from the wall becomes too small, the bot will turn away from the wall and continue moving forward.
If the distance from the wall becomes too big, it will turn back towards the wall to get closer and continue moving forward.

If the bot reads the same 10 values for the front and side values, it will consider itself to be stalled and will try to get out of the stall.

### The Miata+ Algorithm

The Miata+ algorithm builds upon the original Miata wall-following logic with several enhancements:

- **Improved Obstacle Avoidance:** The bot now checks for openings on the right side when an obstacle is detected in front. If an opening is available, it turns right toward the opening instead of defaulting to left turns.
- **Dynamic Stall Recovery:** When the bot detects a stall, it backs up more significantly and uses a sharper angular turn to reposition itself before resuming movement. This helps in scenarios with complex obstacles or tight spaces.
- **Distance-Based Rotation Logic:** The bot tracks the distance traveled since its last rotation and triggers a rotation after traveling a set distance. This ensures more systematic exploration of the environment.
- **Enhanced Wall Following:** The bot dynamically adjusts its speed and turning rate to maintain an optimal distance from the wall. This allows smoother navigation along walls and prevents overcorrection.

These changes make Miata+ more robust in handling obstacles and navigating complex environments while retaining the core wall-following behavior of the original algorithm.

---

### How to Run the Package

#### Prerequisites
Ensure ROS2 Humble and Webots are installed on your system.

#### My Setup
1. Source ROS2 setup:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

#### Running on macOS
1. Set the Webots home environment variable:
   ```bash
   export WEBOTS_HOME=/Applications/Webots.app
   ```
2. Start the local simulation server:
   ```bash
   python3 local_simulation_server.py
   ```

#### Running on Ubuntu
1. Open the first terminal and run the following commands:
   ```bash
   cd f24_robotics
   colcon build
   source install/setup.bash
   source /opt/ros/humble/setup.bash
   ros2 launch miata_hw4 f23_robotics_1_launch.py
   ```

2. Open the second terminal after waiting for the first terminal to be successful in connecting to Webots:
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run miata_hw4 miata_hw4
   ```



Personal note:

Head to /ros2_ws/src/f24_robotics/miata_hw4/csv and in a third terminal
run 
python3 plot.py

but make sure you update the csv name in the plot.py to target the one you want to look at for the plot.