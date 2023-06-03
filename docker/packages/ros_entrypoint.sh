#!/bin/bash
set -e

USERNAME=sierra_miguel_Z621473
TOKEN=N9qVBhZucxzax5iz88uc
DESIRED_BRANCH=$1
GIT_PULL=$2
CHECK_DIR=/home/hmi_volume/hmi

ROS_ENV_SOURCE="/opt/ros/$ROS_DISTRO/setup.bash"

echo "sourcing   $ROS_ENV_SOURCE"
source "$ROS_ENV_SOURCE"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

source ./create_ros_workspace.sh 

export ROS_MASTER_URI=http://192.168.0.37:11311
export ROS_IP=192.168.0.37

BASE_DIR=/home/hmi_volume
WORKSPACE_ENV_SOURCE="$BASE_DIR/hmi/devel/setup.bash"

echo "sourcing   $WORKSPACE_ENV_SOURCE"
source "$WORKSPACE_ENV_SOURCE"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

echo ${WORKSPACE_ENV_SOURCE} >> ~/.bashrc

LOG_DIR=${BASE_DIR}/docker_log

if [[ ! -d  $LOG_DIR ]];
then
  mkdir $LOG_DIR
fi
echo "$(date): hmi server docker started" >> ${LOG_DIR}/server.log

cd /

export ROS_LOG_DIR="$LOG_DIR"

roslaunch launcher hmi_server.launch --wait
