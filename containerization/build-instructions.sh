#!/usr/bin/env bash
: '
WSL Drone Container Build Instructions.
'
set -e

USER_NAME=drone0
USER_UID=1000
USER_GID=1000

VEHICLE=copter
BOARD=sitl

# Install Apt Dependencies
apt-get update && apt-get install --no-install-recommends -y \
	lsb-release gnupg curl tzdata sudo xvfb git python3-pip

# Add Webots Apt Repository
sudo mkdir -p /etc/apt/keyrings && cd /etc/apt/keyrings && sudo wget -q https://cyberbotics.com/Cyberbotics.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" \
| sudo tee /etc/apt/sources.list.d/Cyberbotics.list

# Add Gazebo Apt Repository
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Install Webots and Gazebo
apt-get update && apt-get install --no-install-recommends -y \
	webots gazebo11 libgazebo11-dev

# Create User
groupadd ${USER_NAME} --gid ${USER_GID} \
&& useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash \
&& echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
&& chmod 0440 /etc/sudoers.d/${USER_NAME} \
&& chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

# Install ArduPilot
sudo -u ${USER_NAME} /bin/bash <<-EOSU
	set -e

	cd ~/
	git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

	cd ~/ardupilot
	export SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
	source Tools/environment_install/install-prereqs-ubuntu.sh -y \
	&& source ~/.profile \
	&& ./waf configure --board ${BOARD} \
	&& ./waf ${VEHICLE}

	pip install pymavlink dronekit numpy
EOSU

# Setup WSL
cat > /etc/wsl.conf <<-EOF
	[boot]
	systemd=true
	[user]
	default=${USER_NAME}
EOF

# Clean Image
rm -rvf /var/lib/apt/lists/* /tmp/* /var/tmp/*
apt-get autoremove