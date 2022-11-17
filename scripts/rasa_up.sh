#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/cultureid_user0/catkin_ws/devel/setup.bash

roscd cultureid_exhibit_dialogue/scripts
rasa_output_dirc=$PWD
rasa_output_filename='edassu_docker.txt'
rasa_output_file="${rasa_output_dirc}/${rasa_output_filename}"

echo '[BASH cultureid_exhibit_dialogue] with rasa output file:' $rasa_output_file

# If there exists this `rasa_output_file` then delete it; we want a clean file
ls $rasa_output_dirc | grep $rasa_output_filename
o1=$?
if [ $o1 -ne 0 ]
then
  echo '[BASH cultureid_exhibit_dialogue] rasa output file does not exist; on your way...'
else
  echo '[BASH cultureid_exhibit_dialogue] rasa output file exists; no problem chief, removing it...'
  rm $rasa_output_file
fi

# Execute docker and redirect output to `rasa_output_file`
echo '[BASH cultureid_exhibit_dialogue] FIRING UP RASA'
#docker-compose -f /home/cultureid_user0/rasa-container/docker-compose.yml up --build> $rasa_output_file 2>&1 &
docker-compose -f /home/cultureid_user0/rasa-container/docker-compose.yml up > $rasa_output_file 2>&1 &
