#!/bin/sh
cd /tmp/colcon_ws
. /opt/ros/humble/setup.sh

shopt -s nocasematch

if [[ "$1" = "build" ]]
then
  echo "Rebuilding package robotem_rovne ..."
  cd src/robotem_rovne
  git pull origin main
  cd /tmp/colcon_ws
  colcon build --packages-select robotem_rovne
elif [[ "$1" = "run" ]]
then
  echo "Starting application robotem_rovne"
  . install/setup.sh robotem_rovne
  ros2 launch robotem_rovne all.py
else
  echo "Invalid command option"
fi

shopt -u nocasematch
