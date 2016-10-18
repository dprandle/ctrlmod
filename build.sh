#! /bin/bash 
CONFIG=debug
CONFIG_CMAKE=Debug
BUILD_ALL=NO

cd ~/Documents/code/ctrlmod


build()
{
    mkdir -p build/$CONFIG
    cd build/$CONFIG
    cmake -DCMAKE_BUILD_TYPE=$CONFIG_CMAKE -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ../..
    make -j8
    mv compile_commands.json ../
    cd ..
    rc -J
    cd ..
}

for var in "$@"
do
    if [ "$var" = "-d" ]; then
	CONFIG=debug
	CONFIG_CMAKE=Debug
    elif [ "$var" = "-r" ]; then
	CONFIG=release
	CONFIG_CMAKE=Release
    elif [ "$var" = "-c" ]; then
	rm -r build/$CONFIG
    elif [ "$var" = "-call" ]; then
	rm -r build
    elif [ "$var" = "-all" ]; then
	BUILD_ALL=YES
    fi
done

if [ "$BUILD_ALL" = NO ]; then
    build
else
    CONFIG=debug
    CONFIG_CMAKE=Debug
    build
    CONFIG=release
    CONFIG_CMAKE=Release
    build
fi
