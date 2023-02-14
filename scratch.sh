#!/bin/bash

mkdir "build" || rm -rf "build" && mkdir "build"


cmake -DCMAKE_BUILD_TYPE=Release -B "build"
cmake --build "build"
./build/CSC4140_Assignment2 "$1" "$2" "../$3"
