## 작업 환경
- Ubuntu 22.04
- ROS2 iron
- rmf_demo 2.2.3


## 실행 방법
```bash
# api-server 실행
docker run \
   --network host \
   -it --rm \
   -e ROS_DOMAIN_ID=3 \
   -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   ghcr.io/open-rmf/rmf-web/api-server:latest
# dashboard frontend 실행
docker run \
   --network host -it --rm \
   -e RMF_SERVER_URL=http://localhost:8000 \
   -e TRAJECTORY_SERVER_URL=ws://localhost:8006 \
   ghcr.io/open-rmf/rmf-web/dashboard:latest

# rviz를 포함한 office 런치 파일 실행
export ROS_DOMAIN_ID=3 &&
ros2 launch rmf_demos office.launch.xml \
  server_uri:="ws://localhost:8000/_internal" \
  use_sim_time:=False \
  headless:=False
```

## build 방법
```bash
sudo apt-get install ros-$ROS_DISTRO-tf-transformations
pip3 install fastapi uvicorn websocket-client
python3 -m pip install flask-socketio
sudo apt update && sudo apt install ros-$ROS_DISTRO-rmf-dev
```


## trajectory 죽었을때
```bash
export ROS_DOMAIN_ID=3 &&
ros2 run rmf_visualization_schedule schedule_visualizer_node \
    --ros-args \
    -p use_sim_time:=false \
    -p rate:=10.0 \
    -p path_width:=0.2 \
    -p initial_map_name:=L1 \
    -p wait_secs:=10 \
    -p port:=8006 \
    -p retained_history_count:=50
    
```

## trajectory 확인
- export ROS_DOMAIN_ID=3 && ros2 topic info /schedule_markers -v 확인하면 publisher count가 0이다.



## port 확인
- 8083 7878
- sudo netstat -tuln | grep 8083


## git clone 

- git checkout tags/2.2.3 -b branch-2.2.3


## task 실행
```bash

  
export ROS_DOMAIN_ID=3 &&
ros2 run rmf_demos_tasks dispatch_patrol -p north_east south_east -n 3

export ROS_DOMAIN_ID=3 &&
ros2 run rmf_demos_tasks dispatch_patrol -p turtlebot3_1_charger north_east south_east north_west turtlebot3_1_charger south_west -n 10


export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -p coe pantry lounge supplies hardware_2 -n 10
patrol_D2
patrol_A1
patrol_D1
pantry
lounge
hardware_2
coe
tinyRobot3_charger
patrol_C
patrol_B
supplies
patrol_A2
export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot1 -a patrol_A1 -s coe -n 4

export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot1 -p pantry coe hardware_2 -n 4
export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot2 -p coe pantry lounge -n 4  
export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot3 -p patrol_B patrol_A2 patrol_C -n 4  

# target position 연속 실행
export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks patrol_manager 


export ROS_DOMAIN_ID=13 && 
ros2 run rmf_demos_tasks dispatch_patrol -F DeliveryRobot -R tank_orange -p target_01 target_02 target_03 charger_01 -n 10

export ROS_DOMAIN_ID=13 && 
ros2 run rmf_demos_tasks dispatch_patrol -F DeliveryRobot -R tank_orange -p tmp_03

export ROS_DOMAIN_ID=13 && 
ros2 run rmf_demos_tasks dispatch_patrol -F DeliveryRobot -R steer_yellow -p tmp_03

export ROS_DOMAIN_ID=13 && 
  ros2 run rmf_demos_tasks dispatch_action -F DeliveryRobot -R steer_yellow -a teleop -s tmp_03
export ROS_DOMAIN_ID=13 && 
  ros2 topic pub /action_execution_notice rmf_fleet_msgs/msg/ModeRequest '{fleet_name:  DeliveryRobot, robot_name: steer_yellow, mode: {mode: 0}}' --once

export ROS_DOMAIN_ID=13 && 
  ros2 run rmf_demos_tasks dispatch_action -F DeliveryRobot -R tank_orange -a teleop -s tmp_03
export ROS_DOMAIN_ID=13 && 
  ros2 topic pub /action_execution_notice rmf_fleet_msgs/msg/ModeRequest '{fleet_name:  DeliveryRobot, robot_name: tank_orange, mode: {mode: 0}}' --once


```
