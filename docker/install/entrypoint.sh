#!/bin/bash
set -e

# create zeth network device for zephyr native posix
if [ "$RUN_ZETH" = true ] ; then
  cd /opt/zeth && sudo ./net-setup.sh start
fi

source ~/.profile
source ~/.bashrc

if [ "$RUN_VNC" = true ] ; then
  /opt/TurboVNC/bin/vncserver -geometry 1920x1080 -name cerebri -xstartup /bin/openbox-session :20
fi

exec "$@"
