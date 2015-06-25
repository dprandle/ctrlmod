
cmake_minimum_required(VERSION 2.8)

project(ctrlmod)

# Set the source directory
set(PROJECT_SRC_DIR /home/dprandle/Documents/code/ctrlmod/src)

# Set the include directory
set(PROJECT_INCLUDE_DIR /home/dprandle/Documents/code/ctrlmod/include)

# Set the shared lib directory
set(PROJECT_SHARED_LIB_DIR /home/dprandle/Documents/code/ctrlmod/bin)

# Set the static lib directory
set(PROJECT_STATIC_LIB_DIR /home/dprandle/Documents/code/ctrlmod/lib)

# Set the src files for the project
file(GLOB SRC_FILES "${PROJECT_SRC_DIR}/*.cpp")

# Set project includes dir
include_directories("${PROJECT_INCLUDE_DIR}")

# Set project shared libs dir
link_directories("${PROJECT_SHARED_LIB_DIR}")

# Set project static libs dir
link_directories("${PROJECT_STATIC_LIB_DIR}")

# Adding executable
add_executable(ctrlmod ${SRC_FILES})

