FROM ubuntu:22.04
LABEL maintainer="James Goppert <james.goppert@gmail.com>"

# environment
ENV PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
ENV XDG_RUNTIME_DIR=/tmp/runtime-docker
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV GZ_PARTITION=cognipilot
ENV TERM=xterm-256color
ENV DISPLAY=:20
ENV PATH="/home/user/bin:${PATH}"

# Set default shell during Docker image build to bash
SHELL ["/bin/bash", "-l", "-c"]

# Copy docker clean script
COPY install/docker_clean.sh /docker_clean.sh
RUN chmod +x /docker_clean.sh

# Install base packages
RUN DEBIAN_FRONTEND=noninteractive apt-get -y update && \
	apt-get -y upgrade && \
	apt-get install --no-install-recommends -y \
		sudo \
		locales \
		&& \
	/docker_clean.sh

# Initialise system locale
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
RUN locale-gen en_US.UTF-8

# Create a user to make sure install works without root
ARG UID_INSTALLER=2001
RUN useradd -l -u $UID_INSTALLER installer -G sudo,plugdev && \
 echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER installer

# install dependencies using scripts in a manner that will cache build
# when one script is modified
COPY install/base.sh /tmp/install/base.sh
RUN /tmp/install/base.sh && /docker_clean.sh

COPY install/zephyr.sh /tmp/install/zephyr.sh
RUN /tmp/install/zephyr.sh && /docker_clean.sh

COPY install/gazebo.sh /tmp/install/gazebo.sh
RUN /tmp/install/gazebo.sh && /docker_clean.sh

COPY install/zenoh.sh /tmp/install/zenoh.sh
RUN /tmp/install/zenoh.sh && /docker_clean.sh

COPY install/extra.sh /tmp/install/extra.sh
RUN /tmp/install/extra.sh && /docker_clean.sh

# setup zeth ethernet device
RUN sudo mkdir -p /opt/zeth
COPY install/net-setup.sh /opt/zeth
COPY install/zeth.conf /opt/zeth
RUN sudo chmod +x /opt/zeth/net-setup.sh

# enable apt auto-completion by deleting autoclean task
RUN sudo rm /etc/apt/apt.conf.d/docker-clean

# create XDG runtime dir
RUN mkdir /tmp/runtime-docker && sudo chmod 700 /tmp/runtime-docker

# create a user for running the container
ARG UID_USER=1000
RUN sudo useradd --create-home -l -u $UID_USER -G sudo,plugdev user && \
 echo user: $UID_USER

USER user

COPY install/user_setup.sh /tmp/install/user_setup.sh
RUN /tmp/install/user_setup.sh && /docker_clean.sh

# create workdir, this is where the source code will be mounted
VOLUME /workdir
WORKDIR /workdir
RUN sudo mkdir -p /workdir && sudo chown -R user:user /workdir

# setup entry point
COPY install/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh

CMD ["/bin/bash"]
ENTRYPOINT ["/entrypoint.sh"]

LABEL org.opencontainers.image.source = "https://github.com/CogniPilot/cerebri"

# vim: set et fenc=utf-8 ff=unix ft=dockerfile sts=0 sw=2 ts=2 :
