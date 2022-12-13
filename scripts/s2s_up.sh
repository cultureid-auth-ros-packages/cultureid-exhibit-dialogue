#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/cultureid_user0/catkin_ws/devel/setup.bash

roscd cultureid_exhibit_dialogue/scripts
rasa_output_dirc=$PWD
rasa_output_filename_el='edassu_docker_el.txt'
rasa_output_filename_en='edassu_docker_en.txt'
rasa_output_file_el="${rasa_output_dirc}/${rasa_output_filename_el}"
rasa_output_file_en="${rasa_output_dirc}/${rasa_output_filename_en}"

# Block here until rasa is actually up and running for both languages
while :
do
  cat $rasa_output_file_el | grep 'Rasa server is up and running.'
  if [ $? -ne 0 ]
  then
    sleep 1
  else
    break
  fi
done


# Block here until rasa is actually up and running for both languages
while :
do
  cat $rasa_output_file_en | grep 'Rasa server is up and running.'
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

# The first and only argument should be the port of the rasa server
# (different ports for different, concurrently-running, languages)
python3 send_node.py $1 &
