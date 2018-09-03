## Quick Start:

mkdir -p ~/catkin_ws_mobi_head/src

cd ~/catkin_ws_modi_head/src

git clone https://github.com/alapkshirsagar/mobi_head_gazebo_moveit.git

cd ~/catkin_ws_mobi_head

catkin_make

source devel/setup.bash

roslaunch mobi_head_gazebo mobi_head_gazebo_moveit.launch
