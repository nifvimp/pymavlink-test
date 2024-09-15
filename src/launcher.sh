#!/usr/bin/env bash
source ~/.profile

sim_vehicle.py -w --model webots-python -v copter --add-param-file=$1 --sim-address=$2
