#!/bin/bash

rm -rf build-debug
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE="RelWithDebInfo"
cmake --build build-debug
