#!/bin/bash

# Ros Source 
source /opt/ros/$ROS_DISTRO/setup.sh

if [ ! -d $CHECK_DIR ]; then
    # HMI Clone and Build
    echo "TRYING TO CLONE"
    cd /home/hmi_volume
    git clone https://$USERNAME:$TOKEN@tmc-droom-gitlab.com/kamigo/tugnova/hmi.git
    cd hmi
    git checkout $DESIRED_BRANCH
    catkin_make
else
    if [ $GIT_PULL = true ] ; then
        echo "TRYING TO PULL"
        # HMI Pull, Checkout and Build
        cd /home/hmi_volume/hmi
        git pull
        git checkout $DESIRED_BRANCH
        git pull
        catkin_make
    fi
fi

echo "================HMI CONTAINER COMPILE COMPLETE========================="