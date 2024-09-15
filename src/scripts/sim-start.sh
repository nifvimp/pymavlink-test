#!/bin/bash
# Parse Arguments
usage() {
    echo "Usage: $0 [OPTIONS] world_file param_file"
    echo "Options:"
    echo "  -S <simulation_path>    Specify simulation path (default: ~/ardupilot/Tools/autotest/sim_vehicle.py)"
    echo "  -s <simulation_host>    Specify simulation host (default: 127.0.0.1)"
    echo "  -h <host>               Specify host (default: 127.0.0.1)"
    echo "  -p <port>               Specify port (default: 14550)"
    echo "  -v <vehicle>            Specify vehicle (default: copter)"
    echo "  -m <sim_mode>           Specify simulation mode (default: realtime)"
    echo "  -x <stream_mode>        Specify stream mode (default: x3d)"
    exit 1
}

pos_args=()
while [ $OPTIND -le "$#" ]
do
    if getopts S:s:h:p:v:m:x: opt
    then
        case "$opt" in
            S) sim_vehicle="$OPTARG";;
            s) sim_host="$OPTARG";;
            h) host="$OPTARG";;
            p) port="$OPTARG";;
            v) vehicle="$OPTARG";;
            m) sim_mode="$OPTARG";;
            x) stream_mode="$OPTARG";;
            *) usage;;
        esac
    else
        pos_args+=("${!OPTIND}")
        shift
    fi
done

if [ ${#pos_args[@]} -ne 2 ]; then
    world_file="$WORLD"
    param_file="$PARAM"
else
    world_file="${pos_args[0]}"
    param_file="${pos_args[1]}"
fi

set -m   # Enable Job Control

# Set Defaults
world_file=${world_file:-"$HOME/ardupilot/libraries/SITL/examples/Webots_Python/worlds/iris.wbt"}
param_file=${param_file:-"$HOME/ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm"}
sim_vehicle=${sim_vehicle:-"$HOME/ardupilot/Tools/autotest/sim_vehicle.py"}
vehicle=${vehicle:-"copter"}        # -v
sim_mode=${sim_mode:-"realtime"}    # -m
stream_mode=${stream_mode:-"x3d"}   # -x
simhost=${simhost:-"127.0.0.1"}     # -s
host=${host:-"127.0.0.1"}           # -h
port=${port:-14550}                 # -p
socket="${host}:${port}"

# Execute
shopt -s expand_aliases

source "$HOME"/.ardupilot_env
source "$HOME"/.profile

declare -a pids

xvfb-run --auto-servernum webots --no-rendering --minimize --stdout --stderr --batch \
         --mode="$sim_mode" --stream \
         "$world_file" \
         &
pids=("${pids[@]}" $!)

$sim_vehicle -v "$vehicle" -w --model webots-python \
             --add-param-file="$param_file" \
             --sim-address="$simhost" \
             --out="udp:$socket"

for pid in "${pids[@]}"; do
  kill -9 "$pid"
done