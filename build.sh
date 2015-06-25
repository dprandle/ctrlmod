#! /bin/bash
BV="debug"
cd build
if [ $1 = "-d" ]; then
	cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=~/Documents/code/edison_tc.cmake
else
	cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/Documents/code/edison_tc.cmake
	BV="release"
fi
make
mv /home/dprandle/Documents/code/ctrlmod/build/ctrlmod /home/dprandle/Documents/code/ctrlmod/bin/$BV/ctrlmod