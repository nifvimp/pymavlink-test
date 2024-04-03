#!/bin/bash
# $1 = world name, $2 = param file

#while getopts u:a:f: flag
#do
#    case "${flag}" in
#        u) username=${OPTARG};;
#        a) age=${OPTARG};;
#        f) fullname=${OPTARG};;
#    esac
#done
set -m   # Enable Job Control

vehicle=${vehicle:-"copter"}        # -v
stream_mode=${stream_mode:-"x3d"}   # --stream
sim_mode=${sim_mode:-"realtime"}    # -m

world_file=${world_file:-"$HOME/ardupilot/libraries/SITL/examples/Webots_Python/worlds/iris.wbt"}
param_file=${param_file:-"$HOME/ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm"}
sim_vehicle=${sim_vehicle:-"$HOME/ardupilot/Tools/autotest/sim_vehicle.py"}

simhost=${simhost:-"127.0.0.1"}     # --simhost
host=${host:-"127.0.0.1"}           # -h
port=${port:-14550}                 # -p

socket="${host}:${port}"

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