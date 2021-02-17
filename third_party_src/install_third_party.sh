#!/bin/bash
# install dependencies
sudo apt-get install libuv1-dev libssl-dev unzip

# build uWebSockets library
cd uWebSockets
rm -rf build
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=../../../third_party/uWebSockets ..
make install
cd ..
cd ..

# get & unzip the simulator
wget https://github.com/udacity/self-driving-car-sim/releases/download/v1.45/term2_sim_linux.zip
unzip term2_sim_linux.zip -d ../
rm term2_sim_linux.zip
