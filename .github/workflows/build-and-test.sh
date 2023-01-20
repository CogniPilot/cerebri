#!/bin/bash
set -ev

cd /workdir/cerebri
west init -l .
west update
west build
