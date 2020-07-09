# Obstacle Avoidance Workspace

## Simulation Instructions
- roscore
- rosrun ros_configs ros_configs.py
- rviz
- roslaunch lidar_euclidean_cluster_detect lidar_euclidean_cluster_detect.launch
- roslaunch imm_ukf_pda_track imm_ukf_pda_track.launch
- roslaunch dbw_mkz_can dbw_simulation.launch
- rosrun path_planning path_planner_node
- rosrun motion_planning motion_planner_node
- rosrun control mpc_node
- rostopic pub /vehicle/enable std_msgs/Empty "{}"

## Installation Instructions

### Install ROS-Desktop
- http://wiki.ros.org/ROS/Installation

### Install Dataspeed DBW Simulator
- bash <(wget -q -O - https://bitbucket.org/DataspeedInc/ros_binaries/raw/master/scripts/setup.bash)
- ros-{distro}-dataspeed-dbw-simulator

### Install CPPPAD
- sudo apt-get install gfortran
- sudo apt-get install unzip
- wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
- sudo chmod 755 install_ipopt.sh
- Call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo ./install_ipopt.sh Ipopt-3.12.7`
- sudo apt-get update -y
- sudo apt-get install -y cppad

### Install other prerequisites
- sudo apt-get install ros-{distro}-can-msgs ros-{distro}-geographic-msgs ros-{distro}-gps-common ros-{distro}-derived-object-msgs ros-{distro}-lusb ros-{distro}-jsk-recognition-msgs ros-{distro}-vector-map-msgs ros-{distro}-grid-map-cv ros-{distro}-grid-map-msgs ros-{distro}-grid-map-ros ros-{distro}-jsk-rviz-plugins libeigen3-dev
