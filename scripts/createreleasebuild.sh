#!/bin/bash

rm -rf build-release
cmake -S . -B build-release -DCMAKE_BUILD_TYPE="Release"
cmake --build build-release --parallel 8
 