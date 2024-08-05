#실행 방법
```
우선 git clone 하고 다운 받은 패키지 이름을 dwa_planner로 변경


# 1.모라이 실행

# 2.rosbridge 실행
roslaunch rosbridge_server rosbridge_websocket.launch

# 3.morai 토픽들 실제 장치로 사용하는 토픽으로 변환 런치파일 실행
roslaunch dwa_planner morai_convert.launch

# 4. dwa 런치파일싱행
roslaunch dwa_planner local_planner.launch

# 5. 전역경로 지정 파이썬 파일 실행
rosrun dwa_planner utm_to_move_base_simpl.py

# 6.rviz 켜서 원하는 것 topic 켜서 확인
```

# dwa_planner

[![CI](https://github.com/amslabtech/dwa_planner/workflows/CI/badge.svg)](https://github.com/amslabtech/dwa_planner/actions)
[![issue_opened](https://img.shields.io/github/issues/amslabtech/dwa_planner.svg)](https://github.com/amslabtech/dwa_planner/issues?q=is%3Aopen+is%3Aissue)
[![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/dwa_planner.svg)](https://github.com/amslabtech/dwa_planner/issues?q=is%3Aissue+is%3Aclosed)

## Overview
ROS implementation of DWA(Dynamic Window Approach) Planner

The API documentation is available at [https://amslabtech.github.io/dwa_planner/](https://amslabtech.github.io/dwa_planner/)

**Note: This simulator is not provided.**

![demo 1](docs/images/dwa_planner_demo_1.gif)

## Environment
- Ubuntu 20.04
- ROS Noetic

## Install and Build
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/amslabtech/dwa_planner.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic # Install dependencies
catkin build dwa_planner -DCMAKE_BUILD_TYPE=Release     # Release build is recommended
```

## How to use
```
roslaunch dwa_planner local_planner.launch
```

## Running the demo with docker
```
git clone https://github.com/amslabtech/dwa_planner.git && cd dwa_planner

# build an image, create a container and start demo (Ctrl-c: stop a container and exit)
docker compose up
# remove a container
docker compose down
```

## Running the demo without docker
### Using simulator
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic
catkin build -DCMAKE_BUILD_TYPE=Release

# run demo
export TURTLEBOT3_MODEL=burger
roslaunch dwa_planner demo.launch
```
![demo 2](docs/images/dwa_planner_demo_2.gif)

## Node I/O
![Node I/O](docs/images/dwa_planner_io.png)

### Published/Subscribed Topics
Access [here](docs/Topics.md)

### Runtime requirement
- TF (from GLOBAL_FRAME to ROBOT_FRAME) is required

## Parameters
Access [here](docs/Parameters.md)

## References
- [D. Fox,  W. Burgard, and S.Thrun, "The dynamic window approach to collision avoidance", IEEE Robotics Automation Magazine, 1997.](https://ieeexplore.ieee.org/abstract/document/580977)
