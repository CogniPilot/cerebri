#!/bin/bash
echo "generating rdd2.py"
./rdd2.py
echo "generating rdd2_loglinear.py"
./rdd2_loglinear.py
echo "generating bezier.py"
./bezier.py

pushd ../../../../
west format
popd
