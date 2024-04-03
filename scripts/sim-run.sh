#!/bin/bash
# $1 python main

#while getopts u:a:f: flag
#do
#    case "${flag}" in
#        u) username=${OPTARG};;
#        a) age=${OPTARG};;
#        f) fullname=${OPTARG};;
#    esac
#done

drone_entrypoint="$HOME/vehicle/${drone_entrypoint:-"main.py"}"
pip install -r "$HOME/vehicle/requirements.txt"
python3 "$drone_entrypoint"