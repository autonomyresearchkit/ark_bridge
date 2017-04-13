#!/usr/bin/env bash

cd src/
ln -s -f CMakeLists_server.txt CMakeLists.txt
cd ../
./rosmsg-gen.sh -a
./rosrepub-gen.sh repub_configs/ark_server_republishers.cfg
