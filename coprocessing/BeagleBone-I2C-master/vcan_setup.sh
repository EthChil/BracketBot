#!/bin/sh
sudo modprobe vcan
# Add virtual can device
sudo ip link add dev vcan0 type vcan
# set mtu for CAN FD
sudo ip link set vcan0 mtu 72
# run virtual can device
sudo ifconfig vcan0 up

sudo ./fdcanusb_daemon /dev/fdcanusb vcan0