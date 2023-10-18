#!/bin/bash

source "${PWD}/config.sh"
defhost=$(hostname -I | tr -d '[:blank:]')
host=${host:-$defhost}
socket="${host}:${port}"
echo "will attempt to connect to ${socket}"

$sim_vehicle \
        -v ArduCopter -w --model webots-python \
        --add-param-file=$param_file \
        --sim-address="$host" \
        --out="udp:${socket}"