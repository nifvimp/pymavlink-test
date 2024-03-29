#!/bin/bash
# $1 python main, $2 = world name, $3 = param file

#while getopts u:a:f: flag
#do
#    case "${flag}" in
#        u) username=${OPTARG};;
#        a) age=${OPTARG};;
#        f) fullname=${OPTARG};;
#    esac
#done
set -m # Enable Job Control

stream_mode=${stream_mode:-"x3d"}
sim_mode=${sim_mode:-"realtime"}

drone_entrypoint=${drone_entrypoint:-"$HOME/vehicle/main.py"}

world_file=${world_file:-"$HOME/ardupilot/libraries/SITL/examples/Webots_Python/worlds/iris.wbt"}
param_file=${param_file:-"$HOME/ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm"}
sim_vehicle=${sim_vehicle:-"$HOME/ardupilot/Tools/autotest/sim_vehicle.py"}

simhost=${simhost:-"127.0.0.1"}
host=${host:-"127.0.0.1"}
port=${port:-14550}

socket="${host}:${port}"

declare -a pids

xvfb-run --auto-servernum webots --no-rendering --minimize --stdout --stderr --batch \
         --mode="$sim_mode" --stream \
         "$world_file" \
         &
pids=("${pids[@]}" $!)

$sim_vehicle -v ArduCopter -w --model webots-python \
             --add-param-file="$param_file" \
             --sim-address="$simhost" \
             --out="udp:$socket"

for pid in "${pids[@]}"; do
  kill -9 "$pid"
done