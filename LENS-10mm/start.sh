
#!/bin/bash

if [ -z "$IS_CONTAINER" ]; then
  sudo docker compose down
  sudo docker compose build
  sudo docker compose up -d
  gazebo --verbose ~/LENS-10mm/worlds/iris_arducopter_runway.world &
  sudo docker compose exec drone /bin/bash -ic ./start.sh &
  sudo docker compose exec drone /bin/bash -c \
    "cd scripts &&  ./setup.sh && ./start.sh"
else
  source ~/.profile
  ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter \
    --model=gazebo-iris \
    --sim-address=172.17.0.1 \
    --out=udp:127.0.0.1:60000
fi
