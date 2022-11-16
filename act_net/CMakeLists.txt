cmake_minimum_required(VERSION 3.10)
project(act_net)

set(CMAKE_CXX_STANDARD 14)

# Eigen3
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

# YAML-CPP
find_package(yaml-cpp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} include dependencies/networks_minimal/include)

# Catkin
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
        INCLUDE_DIRS
        ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} include dependencies/networks_minimal/include
        LIBRARIES
        ${PROJECT_NAME}
        ${YAML_CPP_LIBRARIES}
        CATKIN_DEPENDS
)

# Make the whole library
file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS include/*.hpp)
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

# Install
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
        DIRECTORY include/ dependencies/include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)