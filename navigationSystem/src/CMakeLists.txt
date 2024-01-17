# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.10)

# Project's name
project(navSystem)

# Use C++20 standard
set(CMAKE_CXX_STANDARD 20)

# The following folder will be included
include_directories("../../..")

# Eigen header files location
include_directories("C:/Libraries/eigen-3.4.0")

# Add all the cpp files in the navigationSystem/src directory to the build
file(GLOB SOURCES "src/*.hpp"
                  "src/*.cpp"
)

add_executable(navSystem main.cpp quartzAccelerometer.hpp acPowerSupply.hpp airDataComputer.hpp BII_9.hpp blackBox.hpp dcPowerSupply.hpp
                gnss.hpp navigationComputer.hpp navSystemTest.hpp ringLaserGyroscopeV2.hpp systemPowerSupply.hpp
        BII_9.cpp
        quartzAccelerometer.cpp
        acPowerSupply.cpp
        airDataComputer.cpp
        blackBox.cpp
        gnss.cpp
        navigationComputer.cpp
        ringLaserGyroscopeV2.cpp
        systemPowerSupply.cpp)

set_target_properties(navSystem PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)