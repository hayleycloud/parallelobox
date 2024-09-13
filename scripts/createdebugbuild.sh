#!/bin/bash

rm -rf build-debug
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE="RelWithDebInfo"
#cmake -S . -B build-debug -DCMAKE_BUILD_TYPE="RelWithDebInfo" -DCMAKE_CXX_FLAGS="-DUSE_DEBUG=true -DVERBOSE=true"
cmake --build build-debug --parallel 8
