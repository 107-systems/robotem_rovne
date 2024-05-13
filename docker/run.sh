#!/bin/sh
cd /tmp/colcon_ws
. /opt/ros/humble/setup.sh

cd src
git clone https://github.com/107-systems/robotem_rovne
colcon build --packages-select robotem_rovne

cd /tmp/colcon_ws
. install/setup.sh robotem_rovne
ros2 launch robotem_rovne all.py
#ros2 launch pika_spark_bno085_driver imu.py &
