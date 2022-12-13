#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/cultureid_user0/catkin_ws/devel/setup.bash

roscd cultureid_exhibit_dialogue/src/speech_to_text_to_speech/
echo 'Trying to restart rasa'

# The first argument is the language; the second the port
python3 soft_restart_rasa.py $1 $2
echo '[BASH cultureid_exhibit_dialogue] rasa restarted'
