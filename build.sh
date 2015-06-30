#! /bin/bash
cd ~/Documents/code/ctrlmod
if [ $2 = "-c" ]; then
	rm -r build
	mkdir build
fi
BV="debug"
cd build
if [ $1 = "-d" ]; then
	cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=~/Documents/code/edison_tc.cmake
else
	cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/Documents/code/edison_tc.cmake
	BV="release"
fi
make
