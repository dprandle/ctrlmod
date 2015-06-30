
cmake_minimum_required(VERSION 2.8)

project(ctrlmod)

# Set the source directory
set(PROJECT_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# Set the include directory
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

# Set the static lib directory
set(PROJECT_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Set the src files for the project
file(GLOB SRC_FILES "${PROJECT_SRC_DIR}/*.cpp")

# Set project includes dir
include_directories("${PROJECT_INCLUDE_DIR}")

# Adding executable
add_executable(ctrlmod ${SRC_FILES})

target_link_libraries(ctrlmod mraa)