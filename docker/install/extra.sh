#!/bin/bash
set -e

sudo apt-get -y update
sudo apt-get -y upgrade
sudo DEBIAN_FRONTEND=noninteractive  apt-get install --no-install-recommends -y \
	htop \
	iproute2 \
	lcov \
	menu \
	openbox \
	python3-xdg \
	qt5dxcb-plugin \
	screen \
	tcpdump \
	vim

#x11vnc \
#xvfb
