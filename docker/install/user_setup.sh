#!/bin/bash
ZSDK_VERSION="0.15.2"
VNCPASSWD=$1

/opt/toolchains/zephyr-sdk-${ZSDK_VERSION}/setup.sh -c

# create symlink to west in $HOME/bin
mkdir -p ~/bin
cd ~/bin
ln -s /opt/.venv-zephyr/bin/west .

# setup vnc
mkdir ~/.vnc && echo "$VNCPASSWD" | /opt/TurboVNC/bin/vncpasswd -f > ~/.vnc/passwd && \
  chmod 600 ~/.vnc/passwd && \
  openssl req -x509 -nodes -newkey rsa:3702 -keyout ~/.vnc/x509_private.pem -out ~/.vnc/x509_cert.pem -days 3650 -subj '/CN=www.mydom.com/O=My Company Name LTD./C=US'

# setup .profile, note bashrc doesn't get sourced by docker by defualt, .profile does
echo "source /opt/ros/humble/setup.bash" >> ~/.profile
echo "source /opt/ws_ros_gz/install/setup.sh" >> ~/.profile
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.profile
echo "export GZ_SIM_RESOURCE_PATH=/workdir/cerebri/drivers/sim_gz/models:/workdir/cerebri/drivers/sim_gz/worlds" >> ~/.profile

