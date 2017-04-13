#!/usr/bin/env bash

cd src/
ln -s -f CMakeLists_client.txt CMakeLists.txt
cd ../
./rosmsg-gen.sh -a
./rosrepub-gen.sh repub_configs/ark_client_republishers.cfg
