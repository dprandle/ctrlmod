#Are you using the 32 or the 64 bits version of the SDK?.
#set(SDK_32bits FALSE) #Use this line for the 64 bits SDK
set(SDK_32bits TRUE) #Use this line for the 32 bits SDK

#You can change this to any absolute folder in your Edison
#This is the folder where the executable will be copied to.
set(deploymentFolderAbsolutPath home/root/progs)

#Set the target parameters
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 3.10.17-poky-edison+)
set(CMAKE_SYSTEM_PROCESSOR i686)

#Set the host parameters
set(edison_sdk_root /home/dprandle/Documents/edison/i686)
set(cross_compiler_sysroot ${edison_sdk_root}/sysroots/pokysdk)

set(CMAKE_C_COMPILER ${cross_compiler_sysroot}/usr/bin/i586-poky-linux/i586-poky-linux-gcc)
set(CMAKE_CXX_COMPILER ${cross_compiler_sysroot}/usr/bin/i586-poky-linux/i586-poky-linux-g++)	
set(CMAKE_SYSROOT ${edison_sdk_root}/sysroots/core2-32-poky-linux)
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
SET(CMAKE_C_FLAGS "-m32 -march=core2 -mtune=core2 -msse3 -mfpmath=sse -mstackrealign -fno-omit-frame-pointer --sysroot=${CMAKE_SYSROOT} -O2 -pipe -g -feliminate-unused-debug-types")
SET(CMAKE_CXX_FLAGS "-m32 -march=core2 -mtune=core2 -msse3 -mfpmath=sse -mstackrealign -fno-omit-frame-pointer --sysroot=${CMAKE_SYSROOT} -O2 -pipe -g -feliminate-unused-debug-types")

INCLUDE_DIRECTORIES(${CMAKE_SYSROOT}/usr/include)
INCLUDE_DIRECTORIES(${CMAKE_SYSROOT}/usr/include/c++)
INCLUDE_DIRECTORIES(${CMAKE_SYSROOT}/usr/include/c++/i586-poky-linux)
