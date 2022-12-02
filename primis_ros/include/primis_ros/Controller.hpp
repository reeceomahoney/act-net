//
// Created by Reece O'Mahoney
//

#ifndef PRIMIS_ROS_HPP
#define PRIMIS_ROS_HPP

#include <filesystem>
#include <anymal_motion_control/AnymalController.hpp>
#include <loco_anymal/common/CommandTranslator.hpp>

#include "primis/Controller.hpp"


namespace primis_ros {
    class Controller : virtual public anymal_motion_control::AnymalController {

    public:
        using Base = anymal_motion_control::AnymalController;
        using AD = anymal_model::AD;

        Controller();

        ~Controller() override;

        bool create() override;

        bool initialize() override;

        bool advance(anymal_motion_control::Command &command,
                     robot_control::SharedMutex &commandMutex) override;

        bool reset() override;

        bool stop() override { return true; }

        bool preStop() override { return true; }

        anymal_motion_control::SwitchResult
        goToReferenceType(anymal_motion_control::ReferenceType referenceType) override;

        void goToOperationMode(const std::string &operationMode,
                               anymal_motion_control::OperationModeAction *action) override;

        bool loadParameters(const std::string &pathToParameters);

        void updateStateInformation();

    private:
        std::unique_ptr<primis::Controller> primis_;
        std::unique_ptr<loco_anymal::CommandTranslator> commandTranslator_;

    private:
        Eigen::Matrix<double, 19, 1> gc_;
        Eigen::Matrix<double, 18, 1> gv_;

        Eigen::Matrix<double, 4, 1> baseQuat_;
        Eigen::Matrix<double, 3, 3> baseRot_;

        Eigen::Matrix<double, 3, 1> cmdVel_;

        Eigen::Matrix<double, 12, 1> jointPosTargets_;
    };
}


#endif //PRIMIS_ROS_HPP
