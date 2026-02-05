#!/bin/bash
cpu_temp=$(cat /sys/class/thermal/thermal_zone0/temp)
cpu_temp_c=$(echo "scale=1; $cpu_temp/1000" | bc)
echo $cpu_temp_c

