#!/bin/bash

source "${PWD}/config.sh"
defhost=$(hostname -I | tr -d '[:blank:]')
defsimhost=$(cat /etc/resolv.conf | sed -n 's/.*nameserver \([0-9.]*\).*/\1/p')
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