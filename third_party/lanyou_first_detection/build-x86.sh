#!/bin/bash

echo "Building X86 libraries..."

rm -rf build-x86
mkdir -p build-x86
cd build-x86

cmake -DCMAKE_BUILD_TYPE=Debug \
      -DARCH=x86 \
      ..

make -j$(nproc)

echo "Build completed!"
echo "X86 libraries are in: lib_x86/"