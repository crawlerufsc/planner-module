#! /bin/sh
rm -rf build
mkdir  -p build
cd build && cmake ..
make
mv planner-tst ..
