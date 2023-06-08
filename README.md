# ros_project

This is an example node made on ROS1 with Python and C++, to control a Turtlebot3 in a simulation environment using Gazebo. The objective is to create a controller that makes the robot wander while avoiding obstacles on the Real World simulation environment. 

This work has the following requirements.

There are two options to compile the pacakge: Source or Docker.

- [Source](#source)
  - [Source_Requirements](#source_requirements
  - [Source_General_Dependencies](#source_general_dependencies))
  - [Source_Dependencies](#source_dependencies)
  - [Source_Run](#source_run)
- [Docker](#docker)
  - [Docker_Requirements](#docker_requirements)
  - [Docker_Run](#docker_run)



# Source

Compiling from source requires installing all the required packages on the source pc. 


### Source_Requirements

- Ubuntu

Ubuntu Focal 20.04 (LTS)

Ubuntu Bionic 18.04 (LTS)

Please verify that your operative system is running one of the versions above using:

```
lsb_release -a
```

- ROS 1 Melodic or Noetic

Make sure your ROS version is sourced on your `~/.bashrc` file or source it in the terminal you are using by using the following command. Change {ROS-DISTRO} for your targeted version (melodic or noetic).

```
source /opt/ros/<ROS-DISTRO>/setup.bash
```

If ROS1 is not installed in your machine, please install it by following these commands or following the instructions found in 
http://wiki.ros.org/noetic/Installation/Ubuntu

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```

After installing, please make sure that you have required packages to find missing dependencies and to create workspaces.

```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
```

If you are not an advanced user, please make sure you install ROS using the package manager `apt` option and not from source. 
After this please feel free to initialize `rosdep` so that you can install missing dependencies automatically.

```
sudo rosdep init
rosdep update
```

- Python 3

Both Ubuntu 18.04 and 20,04 come with Python 3. Confirm using the following command

```
python3 --version
```

It should be pointing to your desire installation of python.

- Git

Make sure git is installed as it is neccesary to clone this project into your computer.

```
git --version
```

if git is not installed, install it using the following command:

```
sudo apt-get install git
```

### Source_General_Dependencies

Install an extra library to calculate irregular areas

```
sudo apt-get install python3-shapely
```

### Source_Dependencies

First, a few dependencies must be installed using Linux pakage manager `apt`


#### For ROS melodic use this:

```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```
For turtlebot packages use this:

```
sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3
sudo apt-get install ros-melodic-turtlebot3-simulations
sudo apt-get install ros-melodic-turtlebot3-gazebo
```

#### For ROS noetic use this:

```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

For turtlebot packages use this:

```
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-simulations
sudo apt install ros-noetic-turtlebot3-gazebo
```

### Clone repository and compile

Open a terminal in your home directory, create a workspace and clone the repository 

```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/masierra892/ros_project.git
```

Make sure that your version of ROS is sourced in the terminal you are going to use to compile your project

```
source /opt/ros/<ROS-DISTRO>/setup.bash # replace <ROS-DISTRO> with 'melodic' or 'noetic'
```

Go to the directory and compile:

```
cd ~/catkin_ws
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Source_Run.

#### Using the launch files
You have to source the ros workspace where you just compiled the package.
```
source ~/catkin_ws/devel/setup.bash
```
You can launch the Gazebo simulation using the following code in one terminal.
```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

And then use roslaunch to run the controller nodes separately to be able to see all the output
```
rosrun controller_cpp wanderer_controller.py
```
or
```
rosrun controller_cpp controller_cpp
```

#### Using the bash files

If you want to launch the whole system in one terminal you can use the following bash comands. Plaease make sure that the .sh files are executable by typing ll on the scripts folder. If they are not executable, please give them permission by using the following command:

```
chmod +x <sh-file> # Replace the sh-file with the name of each file in the scripts folder. 
```

Then launch the whole project. 

For the main move_base whole package, please use the following command.

```
.~/catkin_ws/ros_project/scripts/move_base_controller_py.sh
```

For the naive controllers, please use this one (Optional)

```
.~/catkin_ws/ros_project/scripts/naive_controller_py.sh
```
or
```
.~/catkin_ws/ros_project/scripts/naive_controller_cpp.sh
```

# Docker

Docker launches the program on a container which makes it easy to install and use on any development environment. In this case there is no need to install any ROS extra dependency.

### Docker_Requirements

- Ubuntu

Docker and docker compose can work on the following systems: 

Ubuntu Lunar 23.04

Ubuntu Kinetic 22.10

Ubuntu Jammy 22.04 (LTS)

Ubuntu Focal 20.04 (LTS)

Ubuntu Bionic 18.04 (LTS)

- Docker

Install docker in case it is not installed on your system.

```
sudo apt-get update
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Confirm the installation using the following command:

```
sudo docker run hello-world
```

After running this command you will have a result stating if the installation was correct or not. Finally there are some post installation commands in order to be able to run docker as a super user. 

```
sudo groupadd docker
sudo usermod -aG docker $USER
```
Then please log out and log in again for the changes to take effect. After this, confirma that docker has superuser rights by doing the previous test without adding sudo at the beginning.

```
docker run hello-world
```
The same result as before should appear. 

- Rocker
Rocker is required to launch docker containers that need to use graphical interfaces such as Rviz or Gazebo. 

```
sudo apt update
sudo apt-get install python3-rocker
```

### Docker_Run.

Open a terminal in your home directory, create a workspace and clone the repository 

```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/masierra892/ros_project.git
```

If you want to launch the whole system in one terminal you can use the following bash comands. Plaease make sure that the .sh files are executable by typing `ll` on the scripts folder terminal. If they are not executable, please give them permission by using the following command:

``
chmod +x <sh-file> # Replace the sh-file with the name of each file in the scripts folder. 
```

Make sure that you have built the custom docker image using:

```
.~/catkin_ws/ros_project/scripts/create_docker_image.sh
```

The installation will take a while. After is done, launch the container.

```
.~/catkin_ws/ros_project/scripts/launch_container.sh
```

All the following steps will be done on the terminal that is insde the container. Go to the directory and compile:

```
cd catkin_ws
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Then launch the whole project. 

For the main move_base whole package, please use the following command.

```
.~/catkin_ws/ros_project/scripts/move_base_controller_py.sh
```



