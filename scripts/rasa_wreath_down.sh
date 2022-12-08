#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/cultureid_user0/catkin_ws/devel/setup.bash

roscd cultureid_exhibit_dialogue/scripts
rasa_output_dirc=$PWD
rasa_output_filename_el='edassu_docker_el.txt'
rasa_output_filename_en='edassu_docker_en.txt'
rasa_output_file_el="${rasa_output_dirc}/${rasa_output_filename_el}"
rasa_output_file_en="${rasa_output_dirc}/${rasa_output_filename_en}"

echo '[BASH cultureid_exhibit_dialogue] will start hosing down rasa'
echo '[BASH cultureid_exhibit_dialogue] removing rasa output files:' $rasa_output_file_el ', ' $rasa_output_file_en

rm $rasa_output_file_el
rm $rasa_output_file_en

echo 'HOSING DOWN RASA'
docker-compose -f /home/cultureid_user0/rasa-container/rasa-container-wreath-el/docker-compose.yml down
docker-compose -f /home/cultureid_user0/rasa-container/rasa-container-wreath-en/docker-compose.yml down

echo '[BASH cultureid_exhibit_dialogue] RASA IS DEAD'
