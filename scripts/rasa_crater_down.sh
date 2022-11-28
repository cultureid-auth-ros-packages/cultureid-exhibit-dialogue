#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/cultureid_user0/catkin_ws/devel/setup.bash

roscd cultureid_exhibit_dialogue/scripts
rasa_output_dirc=$PWD
rasa_output_filename='edassu_docker.txt'
rasa_output_file="${rasa_output_dirc}/${rasa_output_filename}"

echo '[BASH cultureid_exhibit_dialogue] will start hosing down rasa'
echo '[BASH cultureid_exhibit_dialogue] removing rasa output file:' $rasa_output_file

rm $rasa_output_file

echo 'HOSING DOWN RASA'
docker-compose -f /home/cultureid_user0/rasa-container/rasa-container-crater/docker-compose.yml down

echo '[BASH cultureid_exhibit_dialogue] RASA IS DEAD'
