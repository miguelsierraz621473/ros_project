#!/bin/bash
DOCKER_IMAGE_NAME=ros_project_container

script_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd $script_path
cd ..

project_directory=$( echo "$PWD" )
geos_directory=$project_directory/src/geos/cmake

echo $geos_directory

export GEOS_DIR=$geos_directory
echo $GEOS_DIR
catkin_make
