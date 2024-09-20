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
### Prepare simulation
add simulation address to the drone's controller args in `templates/ardupilot-examples/worlds/iris.wbt`.
```
Iris {
  translation 0 0 0.09
  controller "ardupilot_vehicle_controller"
  controllerArgs [
    "--motors"
    "m1_motor, m2_motor, m3_motor, m4_motor"
    "--sitl-address"
    "172.22.84.123"
  ]
  extensionSlot [
  ]
}
```
### Launch simulation
```
python3 src/simulate.py
```

## TODO:
- auto make venv with python 3.8