#!/bin/bash

param_file="$HOME/ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm"
sim_vehicle="$HOME/ardupilot/Tools/autotest/sim_vehicle.py"
host=""
simhost=""
port=14550
defhost=$(hostname -I | awk '{print $1}')
defsimhost=$(grep -w 'host.docker.internal' /etc/hosts | awk '{print $1}')
host=${host:-$defhost}
simhost=${simhost:-$defsimhost}
socket="${host}:${port}"
echo "Currently using ${simhost} as simulation host"
echo "Will attempt to connect to ${socket}"

$sim_vehicle \
        -v ArduCopter -w --model webots-python \
        --add-param-file=$param_file \
        --sim-address=$simhost \
        --out="udp:${socket}"