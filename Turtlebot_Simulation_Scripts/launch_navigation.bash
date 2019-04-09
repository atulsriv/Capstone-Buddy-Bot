cd catkin_ws/
source devel/setup.bash
cd src
export TURTLEBOT3_MODEL=waffle_pi
cd ..
cd ..
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml