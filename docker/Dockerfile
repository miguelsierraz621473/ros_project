FROM ros:noetic-ros-base

#
# Defining entry arguments
#

ENV ROS_DISTRO=noetic
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

RUN apt-get update && apt install -y  ptpd \
                                      libbluetooth-dev \
                                      net-tools \
                                      wireless-tools \
                                      iputils-ping \
                                      ros-${ROS_DISTRO}-gpsd-client \
                                      ros-${ROS_DISTRO}-gps-common \
                                      ros-${ROS_DISTRO}-rosbridge-server \
                                      ros-${ROS_DISTRO}-topic-tools
                        

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
