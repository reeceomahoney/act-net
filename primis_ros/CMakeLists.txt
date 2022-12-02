cmake_minimum_required(VERSION 3.16)
project(primis_ros)

set(CMAKE_CXX_STANDARD 14)

set(CATKIN_PACKAGE_DEPENDENCIES
        anymal_motion_control
        primis
        dynamic_class_loader
        robot_control
        )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES primis_ros
        CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/Controller.cpp
        )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(FILES robot_control_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Plugin ##
#############

find_package(catkin QUIET COMPONENTS dynamic_class_loader ${CATKIN_PACKAGE_DEPENDENCIES})

if (${dynamic_class_loader_FOUND})
    message(Plugin Found)

    add_library(${PROJECT_NAME}_plugin src/robot_control_plugin.cpp)

    target_include_directories(${PROJECT_NAME}_plugin PRIVATE
            include
            ${catkin_INCLUDE_DIRS}
            )

    target_link_libraries(${PROJECT_NAME}_plugin
            ${PROJECT_NAME}
            ${catkin_LIBRARIES}
            )

    install(TARGETS ${PROJECT_NAME}_plugin
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

    install(FILES robot_control_plugin.xml
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
            )

endif (${dynamic_class_loader_FOUND})
