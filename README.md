Clone into your workspace's src folder. Then, in that folder, run:
```bash
cd ../.. && colcon build --packages-select project1phase1
```
Then, to run the code, use the following command:
```bash
ros2 launch project1phase1 gazebo.launch.py
```

To move the robot, in seperate terminal:
```bash
ros2 topic pub -1 /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [10.5, 10.5]}"
```

For Steering:
```bash
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0]}"
```
