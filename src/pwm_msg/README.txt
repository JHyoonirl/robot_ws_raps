1)
bash 파일에 설정해야 하는 것
ROS2

source /opt/ros/humble/setup.bash
source ~/robot_ws_raps/install/local_setup.bash

alias cb='cd ~/robot_ws_raps && colcon build --symlink-install && . install/local_setup.sh'
alias cs='cd ~/robot_ws_raps/src'
alias rt='ros2 topic list'
alias testpub='ros2 run demo_nodes_py talker'
alias testsub='ros2 run demo_nodes_py listener'

2)
pkg update를 위한 설정
apt install setuptools==58.2.0