# Windows Ardupilot-Webots SITL
Collection of helpful scripts to setup an Ardupilot-Webots SITL environment on Windows.

## Setup
### Install WSL2
Install WSL2 by running the following command in powershell as an administrator.

```shell
wsl --install
```
https://learn.microsoft.com/en-us/wsl/install

### Create custom WSL distro
Create a custom WSL distro to host the Ardupilot-Webots SITL environment.

```shell
powershell containerization/create-container.ps1
```

## Launch simulation

To launch a simulation, run the following shell command from the root directory of the project.
### basic example
```shell
python3 src/simulate.py
```

### Computer Vision Example
```shell
python3 src/simulate.py --world templates/ardupilot-examples/worlds/iris_camera_down.wbt --entrypoint src/main_cv.py
```

