#!/bin/bash

rm -rf build-debug
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE="RelWithDebInfo" -DUSE_DEBUG=true -DVERBOSE=true
cmake --build build-debug
