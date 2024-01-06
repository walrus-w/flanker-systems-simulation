# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.10)

# Project's name
project(nav_system)

# Use C++20 standard
set(CMAKE_CXX_STANDARD 20)

# The following folder will be included
include_directories("../..")

# Eigen header files location
include_directories("C:/Libraries/eigen-3.4.0")

# Add all the cpp files in the walrus-new-nav-system directory to the build
file(GLOB SOURCES "walrus-new-nav-system/*.cpp")

# Generate the executable from the sources
add_executable(${PROJECT_NAME} ${SOURCES})