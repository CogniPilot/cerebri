#!/bin/bash
set -e

# create zeth network device for zephyr native posix
if [ "$RUN_ZETH" = true ] ; then
  cd /opt/zeth && sudo ./net-setup.sh start
fi

source ~/.profile
source ~/.bashrc

exec "$@"
