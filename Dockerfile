# Sourced from: https://github.com/ArduPilot/ardupilot/blob/master/Dockerfile
# Sourced from: https://www.youtube.com/watch?v=UEre6Bd75dw
FROM cyberbotics/webots:R2023b-ubuntu22.04
SHELL ["/bin/bash", "-c"]

ARG USER_NAME=vehicle
ARG USER_UID=1000
ARG USER_GID=1000

ARG BOARD=sitl
ARG VEHICLE=copter

RUN apt-get update
RUN apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    tzdata \
    bash-completion \
    rsync \
    iproute2 \
    net-tools \
    git

RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

ENV USER=${USER_NAME}

RUN echo "vehicle ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME} \
    && chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

USER ${USER_NAME}

WORKDIR /home/${USER_NAME}

RUN git config --global http.sslverify false \
    && git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

WORKDIR /home/${USER_NAME}/ardupilot

ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN source Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN echo "alias waf=\"/${USER_NAME}/waf\"" >> ~/.ardupilot_env \
    && echo "PATH=\"\$HOME/.local/bin:\$PATH\"" >> ~/.ardupilot_env

RUN source ~/.profile \
    && ./waf configure --board ${BOARD} \
    && ./waf ${VEHICLE}

ENV PULSE_SERVER=/temp/PulseServer
ENV DISPLAY=:0

ENV VEHICLE_HOME /usr/local/vehicle
ENV PATH /usr/local/vehicle:${PATH}

RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# TODO: put python project setup startup script stuff here
RUN export ENTRYPOINT="/home/vehicle/entrypoint.sh" \
    && echo "#!/bin/bash" > $ENTRYPOINT \
    && echo "set -e" >> $ENTRYPOINT \
    && echo "source ~/.ardupilot_env" >> $ENTRYPOINT \
    && echo "rsync -a $VEHICLE_HOME ~/ --exclude-from=$VEHICLE_HOME/.gitignore" >> $ENTRYPOINT \
    && echo 'exec "$@"' >> $ENTRYPOINT \
    && chmod +x $ENTRYPOINT \
    && sudo mv $ENTRYPOINT "/entrypoint.sh"

WORKDIR /home/${USER_NAME}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
