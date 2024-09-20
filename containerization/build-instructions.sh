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

sudo mkdir -p /etc/apt/keyrings && cd /etc/apt/keyrings && sudo wget -q https://cyberbotics.com/Cyberbotics.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" \
| tee /etc/apt/sources.list.d/Cyberbotics.list

apt-get update && apt-get install --no-install-recommends -y \
    bash-completion lsb-release tzdata sudo \
    git python3-pip \
    webots

groupadd ${USER_NAME} --gid ${USER_GID} \
&& useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash \
&& echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
&& chmod 0440 /etc/sudoers.d/${USER_NAME} \
&& chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

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

cat > /etc/wsl.conf <<-EOF
	[user]
	default=${USER_NAME}
EOF

rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*