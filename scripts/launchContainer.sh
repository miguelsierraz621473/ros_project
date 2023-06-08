#!/bin/bash

rocker --nvidia --x11 --user --volume $HOME/catkin_ws -- ros_project_container:latest