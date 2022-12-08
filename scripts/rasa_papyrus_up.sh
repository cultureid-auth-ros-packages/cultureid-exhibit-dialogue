#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/cultureid_user0/catkin_ws/devel/setup.bash

roscd cultureid_exhibit_dialogue/scripts
rasa_output_dirc=$PWD
rasa_output_filename_el='edassu_docker_el.txt'
rasa_output_filename_en='edassu_docker_en.txt'
rasa_output_file_el="${rasa_output_dirc}/${rasa_output_filename_el}"
rasa_output_file_en="${rasa_output_dirc}/${rasa_output_filename_en}"

echo '[BASH cultureid_exhibit_dialogue] with rasa output files:' $rasa_output_file_el ', ' $rasa_output_filename_en

# If there exists this `rasa_output_file` then delete it; we want a clean file
ls $rasa_output_dirc | grep $rasa_output_filename_el
o1=$?
ls $rasa_output_dirc | grep $rasa_output_filename_en
o2=$?
if [ $o1 -ne 0 ] && [ $o2 -ne 0 ]
then
  echo '[BASH cultureid_exhibit_dialogue] rasa output files do not exist; on your way...'
else
  echo '[BASH cultureid_exhibit_dialogue] rasa output files exist; no problem chief, removing them...'
  rm $rasa_output_file_el
  rm $rasa_output_file_en
fi

# Execute docker and redirect output to `rasa_output_file`
echo '[BASH cultureid_exhibit_dialogue] FIRING UP RASA'
docker-compose -f /home/cultureid_user0/rasa-container/rasa-container-papyrus-el/docker-compose.yml up > $rasa_output_file_el 2>&1 &
docker-compose -f /home/cultureid_user0/rasa-container/rasa-container-papyrus-en/docker-compose.yml up > $rasa_output_file_en 2>&1 &
