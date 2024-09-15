# Windows Ardupilot-Webots SITL
scripts to help setup Ardupilot-Webots SITL environment on Windows.
## Install
- webots: https://cyberbotics.com/doc/guide/installation-procedure#installation-on-windows
- wsl2: https://learn.microsoft.com/en-us/wsl/install
## Setting Up
### Create wsl2 container

```
powershell containerization/create-container.ps1
```
### Launch simulation
```
python3 src/simulate.py
```

## TODO:
- auto make venv with python 3.8