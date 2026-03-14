#!/bin/bash

CAN_IF="can0"
BITRATE=1000000

while true; do
    STATE=$(cat /sys/class/net/$CAN_IF/operstate)
    if [ "$STATE" != "up" ]; then
        echo "$(date) - $CAN_IF is down, restarting..."
        sudo ip link set $CAN_IF down
        sudo ip link set $CAN_IF type can bitrate $BITRATE
        sudo ip link set $CAN_IF up
    fi
    sleep 2   # 每 2 秒检查一次
done
