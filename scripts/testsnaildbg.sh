#!/bin/bash

gdb --args build-debug/parallelobox --in ../Models/Snail.stl --out out --granularity vfine --num 10 --skip-symmetry

