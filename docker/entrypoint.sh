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

exec $@
