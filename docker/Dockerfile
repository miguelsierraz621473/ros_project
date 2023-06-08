FROM ros:${ROS_DISTRO}-ros-base

#
# Defining entry arguments
#

ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

#
# Python Pip Installation
#

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-get install -y python3-pip

#
# Sourcing Ros
#

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc

#
# Ros Dependencies
#

RUN apt-get install -y python3-rosdep \
                       python3-rosinstall \
                       python3-rosinstall-generator \
                       python3-wstool \
                       build-essential \
                       git \
                       python3-rosdep && \
    rosdep update

#
# ROS DEPENDENCIES
#

RUN apt-get update && apt install -y  net-tools \
                                      wireless-tools \
                                      iputils-ping \
                                      python3-shapely \
                                      python3-vcstool


RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-teleop-twist-joy \
                                         ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-laser-proc \
                                         ros-${ROS_DISTRO}-rgbd-launch ros-${ROS_DISTRO}-rosserial-arduino \
                                         ros-${ROS_DISTRO}-rosserial-python ros-${ROS_DISTRO}-rosserial-client \
                                         ros-${ROS_DISTRO}-rosserial-msgs ros-${ROS_DISTRO}-amcl ros-${ROS_DISTRO}-map-server \
                                         ros-${ROS_DISTRO}-move-base ros-${ROS_DISTRO}-urdf ros-${ROS_DISTRO}-xacro \
                                         ros-${ROS_DISTRO}-compressed-image-transport ros-${ROS_DISTRO}-rqt* ros-${ROS_DISTRO}-rviz \
                                         ros-${ROS_DISTRO}-gmapping ros-${ROS_DISTRO}-navigation ros-${ROS_DISTRO}-interactive-markers \
                                         ros-${ROS_DISTRO}-dynamixel-sdk ros-${ROS_DISTRO}-turtlebot3-msgs ros-${ROS_DISTRO}-turtlebot3 \
                                         ros-${ROS_DISTRO}-turtlebot3-simulations ros-${ROS_DISTRO}-turtlebot3-gazebo

                        

#
# Copy Files
#
COPY ./packages/ros_entrypoint.sh /ros_entrypoint.sh
COPY ./packages/create_ros_workspace.sh /create_ros_workspace.sh

#
# Sourcing Ros
#
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc

#
# Setup Entry Point
#
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
