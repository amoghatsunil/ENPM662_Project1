Clone into your workspace's src folder. Then, in that folder, run:
```bash
cd ../.. && colcon build --packages-select project1phase1
```
Then, to run the code, use the following command:
```bash
colcon build
source install/setup.bash
ros2 launch project1phase1 gazebo.launch.py
```

To move the robot, in seperate terminal:
```bash
source install/setup.bash
ros2 run project1phase1 keyboard_control.py
```
