cmake_minimum_required(VERSION 3.16)
project(demoRMR VERSION 1.0 LANGUAGES CXX)

option(DISABLE_OPENCV "Disable OpenCV support" ON)
option(DISABLE_SKELETON "Disable skeleton module" OFF)
option(DISABLE_JOYSTICK "Disable joystick support" ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui Network)

find_package(librobot REQUIRED)

if(NOT DISABLE_JOYSTICK)
    add_subdirectory(QJoysticks-master QJoysticks_build)
endif()

add_subdirectory(demoRMR)