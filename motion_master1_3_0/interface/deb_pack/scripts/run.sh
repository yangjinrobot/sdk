#!/bin/bash
cd /opt/robot/share/motion_master/bin
ulimit -c unlimited

chrt 90 taskset -c 4,5,6,7 ./motion_master
