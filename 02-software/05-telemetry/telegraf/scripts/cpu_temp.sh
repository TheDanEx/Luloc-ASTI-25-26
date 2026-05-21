#!/bin/bash
cpu_temp=$(cat /sys/class/thermal/thermal_zone0/temp)
echo $((cpu_temp / 1000)).$(( (cpu_temp % 1000) / 100 ))
