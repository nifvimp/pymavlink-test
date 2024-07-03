
#!/bin/bash

if [ -z "$IS_CONTAINER" ]; then
  sudo docker compose down
  sudo docker compose build
  sudo docker compose up -d
  gnome-terminal --command="sudo docker compose exec --index 1 -e DRONE_ID=1 drone /bin/bash -ic ./start.sh"
  gnome-terminal --command="sudo docker compose exec --index 2 -e DRONE_ID=2 drone /bin/bash -ic ./start.sh"
#  gnome-terminal --command="sudo docker compose exec --index=1 -e DRONE_ID=1 drone /bin/bash -c \"sleep 10 && cd scripts &&  ./setup.sh && ./start.sh\""
#  gnome-terminal --command="sudo docker compose exec --index=2 -e DRONE_ID=2 drone /bin/bash -c \"sleep 10 && cd scripts &&  ./setup.sh && ./start.sh\""
  gazebo --verbose ~/LENS-10mm/worlds/iris_arducopter_runway.world
else
  source ~/.profile
  ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter \
    --model=gazebo-iris \
    --sim-address=172.17.0.1 \
    --sitl-instance-args="--sim-port-in=$((9001 + $DRONE_ID * 2)) --sim-port-out=$((9000 + $DRONE_ID * 2))" \
    --out=udp:127.0.0.1:60000
fi
