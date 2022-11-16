//
// Created by Siddhant Gangapurwala on 02/02/2022.
//

#include "act_net_ctrl/Controller.hpp"

#include <dynamic_class_loader/export_macros.hpp>
#include <robot_control/controllers/ControllerInterface.hpp>

DYNAMIC_CLASS_LOADER_BEGIN_MANIFEST(robot_control::ControllerInterface)
DYNAMIC_CLASS_LOADER_EXPORT_CLASS(act_net_ctrl::Controller)
DYNAMIC_CLASS_LOADER_END_MANIFEST
