#!/bin/bash

rm -rf build-verbose-release
cmake -S . -B build-verbose-release -DCMAKE_BUILD_TYPE="Release" -DCMAKE_CXX_FLAGS="-DVERBOSE"
cmake --build build-verbose-release --parallel 8
