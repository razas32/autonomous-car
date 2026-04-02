#!/bin/bash

DIR="$(cd "$(dirname "$0")" && pwd)"

if [ "$1" == "1" ]; then
    cp "$DIR/hello_part1.cpp" "$DIR/hello.cpp"
    echo "Switched to Part 1 (RC controller)"
elif [ "$1" == "2" ]; then
    cp "$DIR/hello_part2.cpp" "$DIR/hello.cpp"
    echo "Switched to Part 2 (RPi/MAVLink autonomous)"
else
    echo "Usage: ./switch.sh 1   (RC controller)"
    echo "       ./switch.sh 2   (RPi/MAVLink autonomous)"
    exit 1
fi

echo "Now run: cd /home/ds/Documents/Group-19/PX4-Autopilot && make nxp_fmuk66-v3 && make nxp_fmuk66-v3 upload"
echo "Then run: screen /dev/ttyACM0 57600 8N1"
