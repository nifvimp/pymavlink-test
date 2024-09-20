#!/usr/bin/env bash
shopt -s expand_aliases
source ~/.profile

sim_vehicle.py -w --model=webots-python -v copter --add-param-file=$1
