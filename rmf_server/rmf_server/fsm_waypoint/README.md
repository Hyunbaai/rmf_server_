## voice
- aws polly
```ssml
<speak>
    o
    취소합니다.     
</speak>
```
## build
```bash

sudo apt install python3-pip
sudo apt update && sudo apt install ros-$ROS_DISTRO-rmf-dev
python3 -m pip install flask-socketio
pip3 install fastapi uvicorn websocket-client

mkdir -p ~/ws/3rdparty
cd ~/ws/3rdparty/
python3 --version

# install standalone-smach
git clone https://github.com/gabrielsr/standalone-smach.git standalone-smach && 
    sed -i '247s/isAlive()/is_alive()/g' standalone-smach/smach/concurrence.py &&  
    cd standalone-smach &&  
    sudo python3 setup.py install
    
# install tf-transformations
sudo apt-get install ros-$ROS_DISTRO-tf-transformations
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs

```
## usage
```bash
export ROS_DOMAIN_ID=6 &&
ros2 run fsm_waypoint fsm_waypoint_node
```
