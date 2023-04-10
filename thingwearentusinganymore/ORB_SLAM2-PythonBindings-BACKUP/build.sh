#!/usr/bin/env bash
rm -rf build
mkdir build
cd build
pyenv local 3.8.10
cmake .. -DPYTHON_EXECUTABLE="$(which python)"
make
sudo make install