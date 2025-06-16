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

In order to install, you can use included dockerfile and build it

```
docker build -t bve_autopilot:humble .
```

after that launch the docker alongside the rosbag:

```commandline
xhost +local:docker                                         # allow X11
docker run -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  bve_autopilot:humble
```