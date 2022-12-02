//
// Created by Reece O'Mahoney
//

#include "primis/Controller.hpp"
#include <iostream>

using namespace primis;

Controller::Controller(const std::string &configPath,
                       const std::string &networkParametersPath,
                       const std::string &obMeanPath,
                       const std::string &obVarPath) {
    config_ = YAML::LoadFile(configPath);

    /// Load policy network architecture
    policyLayers_.push_back(config_["policy"]["ob_dim"].as<unsigned int>());

    for (const auto &h: config_["policy"]["hidden_layers"].as<std::vector<unsigned int>>()) {
        policyLayers_.push_back(h);
    }

    policyLayers_.push_back(config_["policy"]["ac_dim"].as<unsigned int>());

    /// Get the activation function
    policyActivationMap_ = {{"relu",       activation.relu},
                            {"tanh",       activation.tanh},
                            {"softsign",   activation.softsign},
                            {"sigmoid",    activation.sigmoid},
                            {"leaky_relu", activation.leakyReLu}};

    /// Create policy network
    policy_ = std::make_unique<MultiLayerPerceptron>(
            policyLayers_,
            policyActivationMap_.at(config_["policy"]["activation"].as<std::string>()),
            networkParametersPath);

    /// initialise position and velocity
    gcInit_ << 0, 0, 0.55, 1.0, 0.0, 0.0, 0.0, 0.0, 0.4, -0.8, 0.0, 0.4, -0.8, 0.0, -0.4, 0.8, 0.0, -0.4, 0.8;
    gc_ = gcInit_;
    gv_.setZero();

    /// observation scaling
    obMean_ = loadParametersFromFile(obMeanPath).transpose().col(0);
    obStd_ = loadParametersFromFile(obVarPath).transpose().col(0).cwiseSqrt();

    /// action scaling
    actionMean_ = gc_.tail(12);
    actionStd_.setConstant(0.5);

    /// initialise containers
    action_ = actionMean_;

    controlDecimation_ = 400/50;

    reset();
}

void Controller::reset() {
    linVel_.setZero();
    angVel_.setZero();

    rotMat_.setIdentity();
    ob_.setZero();
    action_.setZero();

    jointHistory_.setZero();
    jointHistoryQueue_.push(Eigen::Matrix<double, 24, 1>::Zero());

    gc_ = gcInit_;
    gv_.setZero();

    elapsedCallbackSteps_ = 0;
}

const Eigen::Matrix<double, 12, 1> &Controller::step(
        Eigen::Matrix<double, 19, 1> &generalizedCoordinate,
        Eigen::Matrix<double, 18, 1> &generalizedVelocity,
        Eigen::Matrix<double, 3, 1> &command) {

    if (elapsedCallbackSteps_++ % controlDecimation_ != 0) {
        if (elapsedCallbackSteps_ == controlDecimation_) {
            elapsedCallbackSteps_ = 0;
        }

        return action_;
    }

    /// update angular and linear velocity
    gc_ = generalizedCoordinate;
    gv_ = generalizedVelocity;
    quat_[0] = gc_[3]; quat_[1] = gc_[4]; quat_[2] = gc_[5]; quat_[3] = gc_[6];
    quatToRotMat(quat_, rotMat_);
    linVel_ = rotMat_.transpose() * gv_.segment(0, 3);
    angVel_ = rotMat_.transpose() * gv_.segment(3, 3);

    jointAngles_ = gc_.tail(12);
    jointVel_ = gv_.tail(12);

    /// collect observations
    ob_ <<  rotMat_.row(2).transpose(), // body orientation
            jointAngles_, // joint angles
            linVel_, angVel_, // body linear&angular velocity
            jointVel_, // joint velocity
            jointHistoryQueue_.front(), jointHistoryQueue_.back(), // joint history
            action_, // previous action
            command; // command

    /// get action (normalise observation and action)
    ob_ = (ob_ - obMean_).cwiseQuotient(obStd_).cwiseMin(10.).cwiseMax(-10.);
    action_ = policy_->forward(ob_).cwiseMin(2.).cwiseMax(-2.);
    action_ = action_ * actionStd_ + actionMean_;

    /// update joint history
    jointHistory_ << action_ - gc_.tail(12), jointVel_;
    jointHistoryQueue_.pop();
    jointHistoryQueue_.push(jointHistory_);

    return action_;
}

void Controller::quatToRotMat(const Eigen::Matrix<double, 4, 1> &q, Eigen::Matrix<double, 3, 3> &R){
    R(0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    R(1) = 2 * q[0] * q[3] + 2 * q[1] * q[2];
    R(2) = 2 * q[1] * q[3] - 2 * q[0] * q[2];

    R(3) = 2 * q[1] * q[2] - 2 * q[0] * q[3];
    R(4) = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    R(5) = 2 * q[0] * q[1] + 2 * q[2] * q[3];

    R(6) = 2 * q[0] * q[2] + 2 * q[1] * q[3];
    R(7) = 2 * q[2] * q[3] - 2 * q[0] * q[1];
    R(8) = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

const Eigen::MatrixXd &Controller::loadParametersFromFile(const std::string &filePath) {
    /// https://stackoverflow.com/a/22988866

    std::ifstream dataFile;
    dataFile.open(filePath);
    std::string line;
    std::vector<double> values;
    unsigned int rows = 0;

    while (std::getline(dataFile, line)) {
        std::stringstream lineStream(line);
        std::string cell;

        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }

        ++rows;
    }

    fileParameters_ = Eigen::Map<const Eigen::Matrix<typename Eigen::MatrixXd::Scalar,
            Eigen::MatrixXd::RowsAtCompileTime, Eigen::MatrixXd::ColsAtCompileTime,
            Eigen::RowMajor>>(values.data(), rows, static_cast<unsigned int>(values.size()) / rows);

    return fileParameters_;
}

const Eigen::Matrix<double, 19, 1> &Controller::getGcInit() {
    return gcInit_;
}