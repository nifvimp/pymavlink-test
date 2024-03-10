# Sourced from: https://github.com/ArduPilot/ardupilot/blob/master/Dockerfile
FROM ubuntu:22.04
SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000

ARG BOARD=sitl
ARG VEHICLE=copter

WORKDIR /home/vehicle

RUN groupadd vehicle --gid ${USER_GID}\
    && useradd -l -m vehicle -u ${USER_UID} -g ${USER_GID} -s /bin/bash

RUN apt-get update && apt-get install --no-install-recommends -y \
    bash-completion \
    lsb-release \
    sudo \
    tzdata \
    iproute2 \
    git

ENV USER=vehicle

RUN echo "vehicle ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/vehicle \
    && chmod 0440 /etc/sudoers.d/vehicle \
    && chown -R vehicle:vehicle /home/vehicle

USER vehicle

RUN git config --global http.sslverify false \
    && git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

WORKDIR /home/vehicle/ardupilot

ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN source Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN echo "alias waf=\"/${USER_NAME}/waf\"" >> ~/.ardupilot_env \
    && echo "PATH=\"\$HOME/.local/bin:\$PATH\"" >> ~/.ardupilot_env

RUN source ~/.bashrc \
    && ./waf configure --board ${BOARD} \
    && ./waf ${VEHICLE}

#WORKDIR /home/vehicle/src

RUN python3 -m pip install --user --no-deps --no-cache-dir future empy pexpect ptyprocess

WORKDIR /home/vehicle

RUN export ENTRYPOINT="/home/vehicle/entrypoint.sh" \
    && echo "#!/bin/bash" > $ENTRYPOINT \
    && echo "set -e" >> $ENTRYPOINT \
    && echo "source ~/.ardupilot_env" >> $ENTRYPOINT \
    # TODO: put python project setup startup script stuff here
    && echo 'exec "$@"' >> $ENTRYPOINT \
    && chmod +x $ENTRYPOINT \
    && sudo mv $ENTRYPOINT "/entrypoint.sh"

RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# SITL 1
EXPOSE 5760/tcp
EXPOSE 5762/tcp
EXPOSE 5763/tcp
EXPOSE 5501
# SITL 2
EXPOSE 5770/tcp
EXPOSE 5772/tcp
EXPOSE 5511
# SITL 3
EXPOSE 5780/tcp
EXPOSE 5782/tcp
EXPOSE 5783/tcp
EXPOSE 5521
# Multicast
EXPOSE 14550/tcp
EXPOSE 14550/udp

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]