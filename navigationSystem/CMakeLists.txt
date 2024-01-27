cmake_minimum_required(VERSION 3.21)  # Minimum required CMake version

project(simpleNavigationSystem LANGUAGES CXX)  # Replace "YourProjectName" with your actual project name

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add source files to the project
set(SOURCES
        src/main.cpp
        src/acPowerSupply.cpp
        src/airDataComputer.cpp
        src/BII_9.cpp
        src/blackBox.cpp
        src/dcPowerSupply.cpp
        src/gnss.cpp
        src/navigationComputer.cpp
        src/navSystemTest.cpp
        src/quartzAccelerometer.cpp
        src/ringLaserGyroscopeV2.cpp
        src/systemPowerSupply.cpp
)

# Add executable target
add_executable(${PROJECT_NAME} ${SOURCES}
        src/dcPowerSupply.cpp
        src/navSystemTest.cpp)

# Set include directories (if needed)
# target_include_directories(${PROJECT_NAME} PRIVATE path/to/includes)

# Set additional compiler options (if needed)
# target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

# Set additional linker options (if needed)
# target_link_options(${PROJECT_NAME} PRIVATE -lYourLibrary)

# Set output directories for binaries (optional)
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
