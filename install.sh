#!/bin/bash

echo "#############################################"
echo "#### Installing nhk2021_ilias repository ####"
echo "#############################################"

## Resolve dependencies (from git repository)
echo "1/4 Resolve dependencies (from git repository)"
sudo apt-get update
sudo apt-get install python-vcstool
dir_path=`pwd`
dirs=`find $dir_path -maxdepth 2 -type f -name *.rosinstall`
echo $dirs
for dir in $dirs;
do
    echo $dir
    vcs import ../ < $dir
done

## Resolve dependencies (from apt repository)
echo "2/4 Resolve dependencies (from apt repository)"
rosdep update
rosdep install -i --from-paths -y ../roswww
dirs=`find $dir_path -maxdepth 1 -type d`

for dir in $dirs;
do
    echo $dir
    rosdep install -i --from-paths -y $dir
done

sudo apt-get install -y ros-melodic-realsense2-description
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-rplidar-ros

## Give permission to python scripts
echo "3/4 Give permissions to python scripts"
dirs=`find $dir_path -maxdepth 3 -type f -name *.py`

for dir in $dirs;
do
    chmod +x $dir
    echo $dir
done

## Give permission to action msg
chmod +x ./bezier_path_planning_pursuit/action/PursuitPath.action

## install pyrealsense2
sudo apt install python-pip
pip install pyrealsense2

## Build
echo "4/4 catkin_make"
cd ../../
catkin_make

echo "Installing finished successfully."