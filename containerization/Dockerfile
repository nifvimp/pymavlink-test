FROM ubuntu:20.04 AS ardupilot
SHELL ["/bin/bash", "-c"]

ARG USER_NAME=drone-container
ARG USER_UID=1000
ARG USER_GID=1000

ARG VEHICLE=copter
ARG BOARD=sitl

ENV DEBIAN_FRONTEND=noninteractive

# Install Nessessary Packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    bash-completion lsb-release tzdata sudo iproute2 \
    git python3-pip

# Setup Users and Premmisions
RUN groupadd ${USER_NAME} --gid ${USER_GID} \
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME} \
    && chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

WORKDIR /home/${USER_NAME}/
USER ${USER_NAME}

# Clone Ardupilot Repository
RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

WORKDIR /home/${USER_NAME}/ardupilot

# Compile Ardupilot Binaries
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN source Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && source ~/.profile \
    && ./waf configure --board ${BOARD} \
    && ./waf ${VEHICLE}

# Install Common Python Requirements
RUN pip install pymavlink dronekit numpy

 # Cleanup Installs
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /home/${USER_NAME}/
