#!/bin/bash


script_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
CHECK_DIR=

cd $script_path
cd ..

WORKSPACE_DIR=$( echo "$PWD" )
CHECK_DIR=$WORKSPACE_DIR/src/geos

if [ ! -d $CHECK_DIR ]; then
    echo "CLONING REQUIRED LIBRARIES"
    vcs import src < move_base_controller.rosinstall
fi

cd src/geos
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
sudo cmake --build . --target install