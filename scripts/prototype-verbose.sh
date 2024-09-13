#!/bin/bash

rm -rf out/*
build-verbose-release/parallelobox --in ../Models2/snail-50cm.stl --out out --num 20 --granularity vfine

