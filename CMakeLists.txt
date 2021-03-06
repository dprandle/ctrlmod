cmake_minimum_required(VERSION 2.8)

# Include toolchain file
include(${CMAKE_SOURCE_DIR}/Edison.cmake)

project(ctrlmod)

# Executable name
set(exe_name ctrlmod)

# Set the source directory
set(CTRLMOD_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# Set the include directory
set(CTRLMOD_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
add_definitions(-std=c++11)

# Set the src files for the project
file(GLOB SRC_FILES "${CTRLMOD_SRC_DIR}/*.c*")
file(GLOB HEADER_FILES "${CTRLMOD_INCLUDE_DIR}/*.h*")

# Set project includes dir
include_directories("${CTRLMOD_INCLUDE_DIR}")

# Adding executable
add_executable(${exe_name} ${SRC_FILES} ${HEADER_FILES})
set_target_properties(${exe_name} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

# link mraa
target_link_libraries(${exe_name} -lpthread)
