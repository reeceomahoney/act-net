//
// Created by Reece O'Mahoney
//

#include "primis_ros/Controller.hpp"

#include <dynamic_class_loader/export_macros.hpp>
#include <robot_control/controllers/ControllerInterface.hpp>

DYNAMIC_CLASS_LOADER_BEGIN_MANIFEST(robot_control::ControllerInterface)
DYNAMIC_CLASS_LOADER_EXPORT_CLASS(primis_ros::Controller)
DYNAMIC_CLASS_LOADER_END_MANIFEST
