#!/bin/bash

mkdir "build" ||


cmake -DCMAKE_BUILD_TYPE=Release -B "build"
cmake --build "build"
./build/CSC4140_Assignment2 "$@"
