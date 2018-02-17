#!/bin/bash
set -e

mkdir -p build
cd build

cmake .. && make

../local/term2_sim_linux/term2_sim.x86_64 &

./ExtendedKF
