//
// Created by Reece O'Mahoney
//

#include <filesystem>

#include "Eigen/Core"
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "act-net/Controller.hpp"
#include "include/Actuation.hpp"


int main() {
    /// filepaths
    auto path = std::filesystem::current_path().string();
    auto UrdfPath = path + "/../rsc/anymal/urdf/anymal.urdf";
    auto configPath = path +"/../configuration/cfg.yaml";

    auto paramPath = path +"/../rsc/parameters";
    auto networkParametersPath = paramPath + "/full_20000_policy.txt";
    auto obMeanPath = paramPath + "/mean20000.txt";
    auto obVarPath = paramPath + "/var20000.txt";

    auto actuatorPath = paramPath + "/actuation";

    /// create world
    raisim::World world;
    double simulation_dt = 0.001;
    double control_dt = 0.02;
    world.setTimeStep(simulation_dt);

    /// add objects
    auto anymal = world.addArticulatedSystem(UrdfPath);
    anymal->setName("anymal");
    anymal->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    world.addGround();

    /// initialise containers
    Eigen::Matrix<double, 19, 1> gc;
    Eigen::Matrix<double, 18, 1> gv, gf; gf.setZero();
    Eigen::VectorXd gcDyn, gvDyn;
    Eigen::Matrix<double, 12, 1> action;

    ///initialise velocity commands
    Eigen::Vector3d command;
    double fwdVelMax = 1, latVelMax = 0.4, turnVelMax = 1.2;

    /// initialise generators
    std::uniform_real_distribution<double> uniformDist(-1, 1);
    thread_local static std::mt19937 gen;

    /// initialise controller
    Controller controller(configPath,
                          networkParametersPath,
                          obMeanPath,
                          obVarPath);

    /// initialise actuator network
    Actuation actuation(actuatorPath,
                        Eigen::Vector2d{1., 0.1},
                        100.,
                        12);

    /// launch raisim server for visualization. Can be visualized on raisimUnity
    raisim::RaisimServer server(&world);
    server.launchServer();


    /// Reset state
    controller.reset();
    anymal->setState(controller.getGc(), controller.getGv());

    double t = 0;
    double t_max = 10;
    while (t < t_max) {
        anymal->getState(gcDyn, gvDyn);
        gc = gcDyn; gv = gvDyn;

        /// generate commands
        if (int(100*t) % 200 <= 1) {
            command[0] = fwdVelMax * uniformDist(gen);
            command[1] = latVelMax * uniformDist(gen);
            command[2] = turnVelMax * uniformDist(gen);
        }

        /// get action
        action = controller.step(gc, gv, command);
        action = actuation.getActuationTorques(action - gc.tail(12), gv.tail(12));
        gf.tail(12) = action;
        anymal->setGeneralizedForce(gf);

        /// step simulation
        for(int i=0; i< int(control_dt / simulation_dt + 1e-10); i++){
            raisim::MSLEEP(1);
            world.integrate();
        }
        t += control_dt;
    }

    server.killServer();
}