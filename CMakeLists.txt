cmake_minimum_required(VERSION 3.10)
project(primis_raisim)

set(CMAKE_CXX_STANDARD 17)

# Eigen3
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

# YAML-CPP
find_package(yaml-cpp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} include dependencies/networks_minimal/include)

# Make the whole library
file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS include/*.hpp)
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS} test/include/Actuation.hpp)

target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_LIBRARIES})

# Raisim test
list(APPEND CMAKE_PREFIX_PATH $ENV{LOCAL_INSTALL})

find_package(raisim CONFIG REQUIRED)

add_executable(${PROJECT_NAME}_test test/env.cpp)
target_link_libraries(${PROJECT_NAME}_test PUBLIC ${PROJECT_NAME} raisim::raisim pthread)