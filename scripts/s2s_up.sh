#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/cultureid_user0/catkin_ws/devel/setup.bash

roscd cultureid_exhibit_dialogue/scripts
rasa_output_dirc=$PWD
rasa_output_filename='edassu_docker.txt'
rasa_output_file="${rasa_output_dirc}/${rasa_output_filename}"

# Block here until rasa is actually up and running
while :
do
  cat $rasa_output_file | grep 'Rasa server is up and running.'

  if [ $? -ne 0 ]
  then
    sleep 1
  else
    break
  fi
done

echo '[BASH cultureid_exhibit_dialogue] RASA IS RUNNING'
echo '[BASH cultureid_exhibit_dialogue] executing S2S'

# S2S cannot exist without rasa. S2S may now be executed
roscd cultureid_exhibit_dialogue/src/speech_to_text_to_speech
python3 send_node.py &
