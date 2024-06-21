#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

if [ "$(id -u)" != "0" ]; then
  echo "This script must be run as root."
  exit 1
fi

CAN=can0
CAN_BITRATE=250000
# SYSFS_GPIO_NUMBER = ((GPIO_PORT - 1) * 32) + GPIO_PIN
GPIO_CAN0_STBY=160
# BNO085_nIRQ	= MX8MM_IOMUXC_SPDIF_TX_GPIO5_IO3
# BNO085_nRST =	MX8MM_IOMUXC_SAI5_RXD1_GPIO3_IO22
# BNO085_nBOOT = MX8MM_IOMUXC_SAI5_RXD2_GPIO3_IO23
GPIO_NIRQ_NUM=131
GPIO_NRST_NUM=86
GPIO_NBOOT_NUM=87

echo "Configuring $CAN for a bitrate of $CAN_BITRATE bits/s"

function finish
{
  ip link set $CAN down
  echo 1 > /sys/class/gpio/gpio$GPIO_CAN0_STBY/value
  echo $GPIO_CAN0_STBY > /sys/class/gpio/unexport

  echo $GPIO_NIRQ_NUM > /sys/class/gpio/unexport
  echo $GPIO_NRST_NUM > /sys/class/gpio/unexport
  echo $GPIO_NBOOT_NUM > /sys/class/gpio/unexport
}
trap finish EXIT

echo $GPIO_CAN0_STBY > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$GPIO_CAN0_STBY/direction
echo 0 > /sys/class/gpio/gpio$GPIO_CAN0_STBY/value

ip link set $CAN type can bitrate $CAN_BITRATE
ip link set $CAN up
ifconfig $CAN txqueuelen 10

sudo -u fio ifconfig $CAN

echo $GPIO_NIRQ_NUM > /sys/class/gpio/export
echo $GPIO_NRST_NUM > /sys/class/gpio/export
echo $GPIO_NBOOT_NUM > /sys/class/gpio/export

modprobe spidev
chmod ugo+rw /dev/spidev0.0

ifconfig eth0 down
ifconfig eth0 192.168.1.5 netmask 255.255.255.0
ifconfig eth0 up
sudo -u fio ifconfig eth0

docker run -it \
  --ulimit nofile=1024:1024 \
  --rm \
  -u 0 --privileged \
  --device /dev/spidev0.0 \
  -v /sys/class/gpio:/sys/class/gpio \
  -v /tmp:/tmp \
  --network host \
  robotem_rovne_docker bash
