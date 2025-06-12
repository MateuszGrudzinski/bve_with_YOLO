# bve_with_YOLO
 
This repo contains the project, that creates bird's eye view representation of data from sensors of autonomus vehicle.
The visual data aquierd from cameras is used by YOLO network for detecting vehicles, and marking their location
![alt text](example.png)

The code runs with usage of ROS2 Humble, by subscribing to topics form autonomous vehicle
running autoware stack.

BVE code can be run with this command
```
ros2 launch bird_view bve_autopilot_launch.py
```