#!/bin/bash
DOCKER_IMAGE_NAME=ros_project_container

script_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd $script_path
cd ..
cd docker
docker build -t $DOCKER_IMAGE_NAME .