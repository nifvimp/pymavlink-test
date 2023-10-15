# README
## install required packages
Required packages for project. Can be installed using setup.py or downloaded directly using pip.
~~~
pymavlink
numpy
~~~
## Setup Environment
change the constants in `constants.py` and `config.sh` to fit your environment.

It might be possible to merge the files later.
### For `constants.py`
 - `GSC_HOST`: The ip of whatever system is running Ardupilot. (Only like this because I can't figure out how to get
the connection to work any other way. I actually don't know or understand anything about networking stuff.)
 - `GSC_PORT`: The port the ground station will connect through.
~~~
GSC_HOST = '172.22.84.123'
GSC_PORT = '14550'
~~~
Note: `GSC_PORT` does not really need to be changed unless the port is already in use or refuses to connect for 
whatever reason. The `GSC_PORT` for SITL purposes is conventionally `14551` by the way.
### For `config.sh`
 - `param_file`: The absolute path to the param file of the drone that is being simulated in webots.
 - `sim_vehicle`: The absolute path to `sim_vehicle.py` from the Ardupilot repo.
 - `host` : The host of the connection to be added to mavproxy. Actually filling in the variable is optional since 
the script will fill it in automatically with the localhost if it is not specified.
 - `port` : The port the connection to be added to mavproxy will use.

~~~
param_file="$HOME/ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm"
sim_vehicle="$HOME/ardupilot/Tools/autotest/sim_vehicle.py"
host=""
port=14550
~~~

## Launching SITL
just run start.sh in terminal
~~~
./start.sh
~~~

## Notes
 - `start.sh` and `close.sh` only work in linux.
 - `close.sh` can be used to force close ardupilot if you somehow mess up terminating the process and get the 
port locked. This script should be avoided though since it causes some wacky in mavproxy and the terminal.  