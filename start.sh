#!/bin/bash

source "${PWD}/config.sh"
defhost=$(hostname -I | tr -d '[:blank:]')
host=${host:-$defhost}
socket="${host}:${port}"
mavproxy_args="--cmd=\"output add ${socket};\""
echo "will attempt to connect to ${socket}"

$sim_vehicle \
        -v ArduCopter -w --model webots-python \
        --add-param-file=$param_file \
        --sim-address=172.22.80.1 \
        --mavproxy-args="${mavproxy_args}"