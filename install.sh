#!/bin/bash

echo "#############################################"
echo "#### Installing nhk2021_ilias repository ####"
echo "#############################################"

## Resolve dependencies (from git repository)
echo "1/4 Resolve dependencies (from git repository)"
sudo apt install python-vcstool
cd ~/catkin_ws
dir_path="~/catkin_ws/nhk2021_ilias/*"
dirs=`find $dir_path -maxdepth 1 -type f -name *.rosinstall`
for dir in $dirs;
do
    vcs import src < dirs
done

## Resolve dependencies (from apt repository)
echo "2/4 Resolve dependencies (from apt repository)"
cd ~/catkin_ws/src
rosdep install -i --from-paths roswww
dir_path="~/catkin_ws/nhk2021_ilias/*"
dirs=`find $dir_path -maxdepth 0 -type d`

for dir in $dirs;
do
    echo $dir
    rosdep install -i --from-paths dir
done

## Give permission to task_selector.py (in nhk2021_launcher pkg)
echo "3/4 Give permissions to python scripts"
cd ~/catkin_ws/src/nhk2021_ilias
chmod +x ./nhk2021_launcher/scripts/task_selector.py

## Build
echo "4/4 catkin_make"
cd ~/catkin_ws
catkin_make

echo "Installing finished successfully."