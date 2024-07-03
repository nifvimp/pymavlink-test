#!/bin/bash

if [ -z "$DRONE_ID" ]; then
  exit 1
fi
if [ $DRONE_ID -eq 1 ]; then
  python3 test1.py
fi
if [ $DRONE_ID -eq 2 ]; then
  python3 test2.py
fi

