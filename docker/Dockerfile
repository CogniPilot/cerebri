###
### Build: bare minimum to build the binary
###

FROM ubuntu:22.04 AS build
LABEL maintainer="James Goppert <james.goppert@gmail.com>"

# Set default shell during Docker image build to bash
SHELL ["/bin/bash", "-l", "-c"]

COPY docker_clean.sh /docker_clean.sh
RUN chmod +x /docker_clean.sh

# Install base packages
RUN apt-get -y update && \
	apt-get -y upgrade && \
	apt-get install --no-install-recommends -y \
		sudo \
		locales \
		&& \
	/docker_clean.sh

# set locale
RUN locale-gen en_US.UTF-8

# set user as default so install scripts look the same for local host install
RUN groupadd -r -g 110 render \
 && groupadd -r -g 107 input \
 && adduser --disabled-password --gecos '' user \
 && adduser user sudo \
 && adduser user render \
 && adduser user input \
 && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER user

# initialise system locale
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
# required to allow nanopb to build on ubuntu 22.04 currently
ENV PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
ENV XDG_RUNTIME_DIR=/tmp/runtime-docker

# install dependencies using scripts in a manner that will cache build
# when one script is modified
COPY install/base.sh /tmp/install/base.sh
RUN /tmp/install/base.sh && /docker_clean.sh

COPY install/zephyr.sh /tmp/install/zephyr.sh
RUN /tmp/install/zephyr.sh && /docker_clean.sh

COPY install/ros.sh /tmp/install/ros.sh
RUN /tmp/install/ros.sh && /docker_clean.sh

COPY install/gazebo.sh /tmp/install/gazebo.sh
RUN /tmp/install/gazebo.sh && /docker_clean.sh

COPY install/zenoh.sh /tmp/install/zenoh.sh
RUN /tmp/install/zenoh.sh && /docker_clean.sh

COPY install/extra.sh /tmp/install/extra.sh
RUN /tmp/install/extra.sh && /docker_clean.sh

# enable apt auto-completion by deleting autoclean task
RUN sudo rm /etc/apt/apt.conf.d/docker-clean

# create setting directory for gazebo
RUN mkdir -p /home/user/.gz \
  && chown -R user:user /home/user/.gz

# create XDG runtime dir
RUN mkdir /tmp/runtime-docker && sudo chmod 700 /tmp/runtime-docker

# create workdir
VOLUME workdir
RUN sudo mkdir -p /workdir && sudo chown -R user:user /workdir

# build custom ros gz for joystick
RUN mkdir -p /home/user/ws_ros_gz/src && \
  cd /home/user/ws_ros_gz/src && \
  git clone -b pr-bring-some-joy-back https://github.com/rudislabs/ros_gz.git && \
  cd /home/user/ws_ros_gz && \
  source /opt/ros/humble/setup.bash && \
  colcon build

# setup .profile, note bashrc doesn't get sourced by docker by defualt, .profile does
RUN mkdir -p /home/user/.bash_completion.d && \
  echo "source /opt/ros/humble/setup.bash" >> ~/.profile && \
  echo "source /home/user/ws_ros_gz/install/setup.sh" >> ~/.profile && \
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.profile &&  \
  echo "export GZ_SIM_RESOURCE_PATH=/workdir/cerebri/drivers/sim_gz/models:/workdir/cerebri/drivers/sim_gz/worlds" >> ~/.profile

# setup zeth ethernet device
COPY install/net-setup.sh /tmp
COPY install/zeth.conf /tmp
RUN sudo chmod +x /tmp/net-setup.sh

# enable entrypoint
ENV PATH="/home/user/bin:${PATH}"
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV GZ_PARTITION=cognipilot
ENV TERM=xterm-256color

WORKDIR /workdir
COPY entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# vim: set et fenc=utf-8 ff=unix ft=dockerfile sts=0 sw=2 ts=2 :
