FROM ubuntu:20.04
SHELL ["/bin/bash", "-c"]

ARG USER_NAME=drone0
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install tzdata sudo

RUN groupadd ${USER_NAME} --gid ${USER_GID} \
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME} \
    && chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

WORKDIR /home/${USER_NAME}/
USER ${USER_NAME}

RUN sudo apt-get update && sudo apt-get install -y git
RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

WORKDIR /home/${USER_NAME}/ardupilot

RUN SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1 \
    && source Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && source ~/.profile \
    && ./waf configure --board sitl \
    && ./waf copter

RUN  sudo passwd -d ${USER_NAME}

ADD scripts /home/${USER_NAME}/scripts/
COPY start.sh /home/${USER_NAME}/
ENV IS_CONTAINER=1

WORKDIR /home/${USER_NAME}/
