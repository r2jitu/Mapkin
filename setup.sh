#!/bin/bash

cd libfreenect
mkdir build
cd build
cmake ..
make
cd ../..
make
