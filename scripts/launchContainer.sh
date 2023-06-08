#!/bin/bash

rocker --nvidia --x11 --user --volume $HOME/ros_project -- ros_project_container:latest