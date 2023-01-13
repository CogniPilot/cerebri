#!/bin/bash
set -e

# create zeth network device for zephyr native posix
cd /tmp && sudo ./net-setup.sh start

# fix docker device ownership
sudo chown root:input /dev/input/*

# move to workdir
cd /workdir/

source ~/.profile
source ~/.bashrc

# setup vnc
#SNUM=$(echo $DISPLAY | sed 's/:\([0-9][0-9]*\)/\1/')
#xvfb-run -n $SNUM -s "-screen 0 1024x768x24" -f ~/.Xauthority openbox-session > /dev/null &
#sleep 1
#x11vnc -display $DISPLAY -usepw -forever -quiet &

exec "$@"
