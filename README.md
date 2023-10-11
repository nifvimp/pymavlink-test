# README
## install required packages
enter one of the following commands into the environment terminal to setup the environment.
~~~
pip install .
~~~
~~~
python setup.py install
~~~
All these are essenially doing is installing pymavlink and other dependencies through `setup.py`. These instructions
are pointless if you are currently running the code if you aren't using a virtual environment and already have the 
dependinces installed.
## Setup Development Environment
change the constants in `constants.py` to fit your environment.

`GSC_HOST`: The ip of whatever system is running Ardupilot. (Only like this because I can't figure out how to get
the connection to work any other way. I actually don't know or understand anything about networking stuff.)

`GSC_PORT`: The port the ground station will connect through.
~~~
GSC_HOST = '172.22.84.123'
GSC_PORT = '14550'
~~~
Note: `GSC_PORT` does not really need to be changed unless the port is already in use or refuses to connect for 
whatever reason. The `GSC_PORT` for SITL purposes is conventionally `14551` by the way.
## Configuring Ardupilot
After running the `sim_vehicle.py` and connecting to webots, type output into the console. It should output 
like the example below:
~~~
output
STABILIZE> 1 outputs
0: 172.22.84.123:14550
1: 172.22.84.123:14551
~~~
If you are missing ports, just add them.
~~~
output add 172.22.84.123:14550
~~~
Please remember that you have to re-add your output port each time you launch `sim-vehicle.py`.
