cmake_minimum_required(VERSION 3.16)

project(canbus_simulator)

set(SOURCES
    src/main.cpp
    src/canbus.cpp
    src/sensornode.cpp
    src/controllernode.cpp
)

include_directories(include)

add_executable(canbus_simulator ${SOURCES})

enable_testing()
add_subdirectory(tests)
