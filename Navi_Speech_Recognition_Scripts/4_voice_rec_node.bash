cd ../buddybot_ws/
source devel/setup.bash
cd src

rosrun navi_speech voice_control_example.py dict:=/Capstone-Buddy-Bot/buddybot_ws/src/navi_speech/examples/voice_cmd.dic kws:=/Capstone-Buddy-Bot/buddybot_ws/src/navi_speech/examples/voice_cmd.kwlist
