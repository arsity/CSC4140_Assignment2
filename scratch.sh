#!/bin/bash

mkdir "build" || rm -rf "build" && mkdir "build"
mkdir "result_img" || rm -rf "result_img" && mkdir "result_img"

cmake -DCMAKE_BUILD_TYPE=Release -B "build"
cmake --build "build"
./build/CSC4140_Assignment2
