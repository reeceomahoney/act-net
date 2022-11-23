//
// Created by Reece O'Mahoney
//

#ifndef ACT_NET_CONTROLLER_HPP
#define ACT_NET_CONTROLLER_HPP

#include "Eigen/Core"
#include <queue>
#include <memory>
#include <unordered_map>

#include "networks_minimal/MultiLayerPerceptron.hpp"
#include <yaml-cpp/yaml.h>


class Controller {

public:
    Controller(
            const std::string &configurationPath,
            const std::string &networkParametersPath,
            const std::string &stateNormalizationOffsetPath,
            const std::string &stateNormalizationScalingPath);

    void reset();

    const Eigen::Matrix<double, 12, 1> &step(
            Eigen::Matrix<double, 19, 1> &generalizedCoordinate,
            Eigen::Matrix<double, 18, 1> &generalizedVelocity,
            Eigen::Matrix<double, 3, 1> &command);

    static void quatToRotMat(const Eigen::Matrix<double, 4, 1> &q, Eigen::Matrix<double, 3, 3> &R);

    const Eigen::MatrixXd &loadParametersFromFile(const std::string &filePath);

    const Eigen::Matrix<double, 19, 1> &getGc() const;

    const Eigen::Matrix<double, 18, 1> &getGv() const;

private:
    Eigen::Matrix<double, 96, 1> ob_, obMean_, obStd_;
    Eigen::Matrix<double, 19, 1> gc_, gcInit_;
    Eigen::Matrix<double, 18, 1> gv_;

    Eigen::Matrix<double, 12, 1> action_, actionMean_;
    Eigen::Matrix<double, 1, 1> actionStd_;

    Eigen::Matrix<double, 4, 1> quat_;
    Eigen::Matrix<double, 3, 3> rotMat_;

    Eigen::Matrix<double, 3, 1> linVel_, angVel_;

    Eigen::Matrix<double, 12, 1> jointAngles_;
    Eigen::Matrix<double, 12, 1> jointVel_;

    std::queue<Eigen::Matrix<double, 24, 1>> jointHistoryQueue_;
    Eigen::Matrix<double, 24, 1> jointHistory_;

    std::unique_ptr<MultiLayerPerceptron> policy_;
    std::vector<unsigned int> policyLayers_;
    std::unordered_map<std::string, std::reference_wrapper<Activation>> policyActivationMap_;

    Eigen::MatrixXd fileParameters_;

    YAML::Node config_;
};


#endif //ACT_NET_CONTROLLER_HPP
