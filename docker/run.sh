#!/bin/sh
cd /tmp/colcon_ws
. /opt/ros/humble/setup.sh

if [ "$1" = "build" ]
then
  echo "Rebuilding package robotem_rovne ..."
  cd src/robotem_rovne
  git pull origin main --rebase
  cd /tmp/colcon_ws
  colcon build --packages-select robotem_rovne
elif [ "$1" = "run" ]
then
  echo "Starting application robotem_rovne"
  cd /tmp/colcon_ws
  . install/setup.sh
  ros2 launch robotem_rovne all.py
else
  echo "Invalid command option"
fi
