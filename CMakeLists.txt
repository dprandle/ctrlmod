cmake_minimum_required(VERSION 2.8)

project(ctrlmod)

# Set the source directory
set(PROJECT_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# Set the include directory
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

# Set the static lib directory
set(PROJECT_STATIC_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/bin)

# Set the library output path
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Set the src files for the project
file(GLOB SRC_FILES "${PROJECT_SRC_DIR}/*.c*")

# Set project includes dir
include_directories("${PROJECT_INCLUDE_DIR}")

# Adding executable
add_executable(ctrlmod ${SRC_FILES})
set_target_properties(ctrlmod PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

# link mraa
target_link_libraries(ctrlmod mraa)
