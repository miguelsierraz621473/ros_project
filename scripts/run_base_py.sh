#!/bin/bash

script_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd $script_path
cd ..

source devel/setup.bash

export TURTLEBOT3_MODEL=burger

roslaunch project_launcher project_launcher_py.launch