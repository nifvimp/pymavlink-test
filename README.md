# Directions
## Requirements
- windows 11+
- wsl installed (ubuntu image)
- Docker installed
```
http://localhost:1234/index.html
```
## Starting the Simulation
first, run the following command to start the docker and simulation:
```
docker-compose up
```
then, run `./sim.ps1` on Windows, or `./sim.sh` on Unix based systems to deploy and run your code.

finally, go to the following url to see the simulation:
```
http://localhost:1234/index.html
```
### SIM
```
sim [OPTIONS] drone-script worldfile param-file
... 
```
## NOTES
Try not to do `docker-compose down` to stop the docker since the docker will have to recompile ardupilot.