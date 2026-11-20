#!/bin/bash
cd /opt/robot/share/motion_master/bin
sudo chmod +x ./motion_master
ulimit -c unlimited
sudo bash -c "echo '/var/opt/robot/log/system/coredump.%e.%p.%t' > /proc/sys/kernel/core_pattern"

chrt 90 taskset -c 4,5,6,7 ./motion_master