#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

if [ "$(id -u)" != "0" ]; then
  echo "This script must be run as root."
  exit 1
fi

if [ "$#" -ne 1 ]; then
  echo "Usage: sudo ./docker-run.sh [build|run]"
  exit 1
fi

CAN=can0
CAN_BITRATE=250000
GPIO_CAN0_STBY=160

echo "Configuring $CAN for a bitrate of $CAN_BITRATE bits/s"

function finish
{
  ip link set $CAN down
  echo 1 > /sys/class/gpio/gpio$GPIO_CAN0_STBY/value
  echo $GPIO_CAN0_STBY > /sys/class/gpio/unexport
}
trap finish EXIT

echo $GPIO_CAN0_STBY > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$GPIO_CAN0_STBY/direction
echo 0 > /sys/class/gpio/gpio$GPIO_CAN0_STBY/value

ip link set $CAN type can bitrate $CAN_BITRATE
ip link set $CAN up
ifconfig $CAN txqueuelen 1000

sudo -u fio ifconfig $CAN

docker run -it \
   -u 0 \
   --privileged \
   --network host \
   robotem_rovne_docker $1
