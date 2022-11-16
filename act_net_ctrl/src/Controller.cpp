//
// Created by Reece O'Mahoney
//

#include "act_net_ctrl/Controller.hpp"

#include <anymal_motion_control/checks/ContactStateCheck.hpp>
#include <anymal_motion_control/checks/StateStatusCheck.hpp>
#include <anymal_motion_control/checks/JointPositionLimitsCheck.hpp>
#include <anymal_motion_control/checks/BaseStateCheck.hpp>

#include <loco/common/ParameterSet.hpp>
#include <memory>

using namespace act_net_ctrl;

Controller::Controller() = default;

Controller::~Controller() = default;

bool Controller::reset() {
    return Controller::initialize();
}

bool Controller::create() {
    getInitialStateChecker().addStateCheck("stateStatus", std::make_shared<anymal_motion_control::StateStatusCheck>());

    getInitialStateChecker().addStateCheck("contactState", std::make_shared<anymal_motion_control::ContactStateCheck>(
            anymal_motion_control::ContactStateCheck::ContactState::CLOSED,
            anymal_motion_control::ContactStateCheck::CheckMode::EQUAL, 4));
    anymal_motion_control::JointPositionLimitsCheck::JointPositionLimits jointPositionLimits;
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::LF_HAA)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-35.0 / 180 * M_PI,
                                                                                   35.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::LF_HFE)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(5.0 / 180 * M_PI, 85.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::LF_KFE)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-85.0 / 180 * M_PI,
                                                                                   -5.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::RF_HAA)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-35.0 / 180 * M_PI,
                                                                                   35.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::RF_HFE)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(5.0 / 180 * M_PI, 85.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::RF_KFE)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-85.0 / 180 * M_PI,
                                                                                   -5.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::LH_HAA)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-35.0 / 180 * M_PI,
                                                                                   35.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::LH_HFE)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-85.0 / 180 * M_PI,
                                                                                   -5.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::LH_KFE)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(5.0 / 180 * M_PI, 85.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::RH_HAA)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-35.0 / 180 * M_PI,
                                                                                   35.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::RH_HFE)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-85.0 / 180 * M_PI,
                                                                                   -5.0 / 180 * M_PI);
    jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(
            anymal_description::AnymalDescription::JointEnum::RH_KFE)] =
            std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(5.0 / 180 * M_PI, 85.0 / 180 * M_PI);
    getInitialStateChecker().addStateCheck("jointPositionLimits",
                                           std::make_shared<anymal_motion_control::JointPositionLimitsCheck>(
                                                   jointPositionLimits));
    getStateChecker().addStateCheck("stateStatus", std::make_shared<anymal_motion_control::StateStatusCheck>());

#if ROS_VERSION_MINIMUM(1, 15, 0)  // if ROS version >= ROS_NOETIC
    getStateChecker().addStateCheck("baseState",
                                    std::make_shared<anymal_motion_control::BaseStateCheck>(20.0, 20.0, 1.0));
#else
    getStateChecker().addStateCheck("baseState", std::make_shared<anymal_motion_control::BaseStateCheck>(20.0, 20.0));
#endif

    commandTranslator_ = std::make_unique<loco_anymal::CommandTranslator>();

    // Add control modes
    setAvailableOperationModesForReferenceType(
            anymal_motion_control::ReferenceType::TWIST, std::vector<std::string>{"walk"});

    return true;
}

bool Controller::initialize() {
    // Create Objects
    std::string actNetParameters = ros::package::getPath("act_net_ctrl") + "/parameters";

    actNet_ = std::make_unique<act_net::Controller>(
            actNetParameters + "/agent.yaml",
            actNetParameters + "/policy.txt",
            actNetParameters + "/mean.txt",
            actNetParameters + "/var.txt"
    );

    // Reset objects
    actNet_->reset();
    jointPosTargets_ = actNet_->getGcInit().tail(12);

    return true;
}

bool Controller::loadParameters(const std::string &configPath) {
    //load parameter file
    loco::ParameterSet parameterSet;
    parameterSet.loadXmlDocument(configPath + "/gains.xml");

    TiXmlHandle parameterHandle = parameterSet.getHandle();

    if (!commandTranslator_->loadParameters(parameterHandle, "ActuationGains")) {
        MELO_ERROR("Could not load Actuation Gains.")
        return false;
    }

    return true;
}

bool Controller::advance(anymal_motion_control::Command &command, robot_control::SharedMutex &commandMutex) {
    updateStateInformation();
    jointPosTargets_ = actNet_->step(gc_, gv_, cmdVel_);

    /// Set the actuation commands
    boost::unique_lock<boost::shared_mutex> lockCommand(commandMutex);
    commandTranslator_->setPidGains(command);

    for (const auto actuatorEnum: std_utils::enum_iterator<AD::ActuatorEnum>()) {
        command.getActuatorCommands()[actuatorEnum].setMode(
                series_elastic_actuator::SeActuatorCommand::SeActuatorMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS);
        command.getActuatorCommands()[actuatorEnum].setJointPosition(
                jointPosTargets_[static_cast<uint>(actuatorEnum)]);
        command.getActuatorCommands()[actuatorEnum].setJointVelocity(0.);
        command.getActuatorCommands()[actuatorEnum].setJointTorque(0.);
    }

    return true;
}

void Controller::updateStateInformation() {
    // Get generalized coordinates
    gc_ = getState().getAnymalModelPtr()->getState().getGeneralizedCoordinates();

    baseQuat_ = gc_.segment(3, 4);
    actNet_->quatToRotMat(baseQuat_, baseRot_);

    // Get generalized velocities
    gv_ = getState().getAnymalModelPtr()->getState().getGeneralizedVelocities();
    gv_.segment(3, 3) = baseRot_ * gv_.segment(3, 3);

    // Get velocity command
    cmdVel_.head(2) = getState().getDesiredRobotVelocityPtr()->getVector().head(2);
    cmdVel_[2] = getState().getDesiredRobotVelocityPtr()->getVector()[5];
}

anymal_motion_control::SwitchResult Controller::goToReferenceType(anymal_motion_control::ReferenceType referenceType) {
    switch (referenceType) {
        case (anymal_motion_control::ReferenceType::TWIST): MELO_INFO("[ActNet] Set Twist reference type.")
            break;

        case (anymal_motion_control::ReferenceType::ACTION): MELO_WARN(
                    "[ActNet] Action reference type is not implemented.")
            return anymal_motion_control::SwitchResult::ERROR;

        case (anymal_motion_control::ReferenceType::POSE): MELO_WARN(
                    "[ActNet] Action reference type is not implemented.")
            return anymal_motion_control::SwitchResult::ERROR;

        case (anymal_motion_control::ReferenceType::NA): MELO_WARN("[ActNet] Reference type is not available.")
            return anymal_motion_control::SwitchResult::ERROR;
    }

    return anymal_motion_control::SwitchResult::SWITCHED;
}


void Controller::goToOperationMode(
        const std::string &operationMode, anymal_motion_control::OperationModeAction *action) {

    if (operationMode == "walk") {
        initialize();

        action->setSucceeded(
                anymal_motion_control::SwitchResult::SWITCHED,
                "Successfully switched operation mode to walk"
        );

        return;
    }

    action->setAborted(anymal_motion_control::SwitchResult::ERROR, "Could not execute the desired operation mode");
}