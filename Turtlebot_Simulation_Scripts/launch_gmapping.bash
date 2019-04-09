cd catkin_ws/
source devel/setup.bash
cd src
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping