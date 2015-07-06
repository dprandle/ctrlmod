#! /bin/bash
BV="debug"

if [ $2 = "-c" ]; then
    rm -r ~/Documents/code/ctrlmod/build
    mkdir ~/Documents/code/ctrlmod/build
fi

cd ~/Documents/code/ctrlmod/build
if [ $1 = "-d" ]; then
	cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=~/Documents/code/edison_tc.cmake
else
	cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/Documents/code/edison_tc.cmake
	BV="release"
fi

make
