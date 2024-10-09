#!/usr/bin/env bash

# Necessary to setup env
shopt -s expand_aliases
source ~/.profile

# Launch Ardupilot SITL
sim_vehicle.py -w --model=webots-python -v ArduCopter --mavproxy-args=--daemon \
	--out=127.0.0.1:14550 \
	--add-param-file=$1
