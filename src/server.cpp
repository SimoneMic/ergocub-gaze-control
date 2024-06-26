/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <thread>
#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Property.h>
#include "eCubGazeControllerInterface/eCubGazeControllerInterface.h"
#include "GazeControl.h"

class RPCServer: eCubGazeControllerInterface{
public:
    GazeControl* gazeController;

    RPCServer(GazeControl* gazeControl){
        this->gazeController = gazeControl;
    };

    bool open(){
        this->yarp().attachAsServer(server_port);
        if (!server_port.open("/Components/GazeController")) {
            yError("Could not open ");
            return false;
        }
        return true;
    }

    void close(){
        server_port.close();
    }

    bool look_at(const std::vector<double>& point){
        std::vector<double> gaze(point);
        Eigen::Map<Eigen::VectorXd> eigen_gaze(&gaze[0], 3);
        this->gazeController->set_gaze(eigen_gaze);
        return true;
    }

    bool set_gain(const double gain){
        return this->gazeController->set_cartesian_gains(gain);
    }

    bool set_motor_actuation(const bool enabled){
        gazeController->set_motor_actuation(enabled);
        return true;
    }

private:
    yarp::os::RpcServer server_port;
};

int main(int argc, char *argv[]){
    std::string config_file = "config.ini";
    
    if (argc == 2){
        config_file = argv[1];
    }

    yarp::os::Property prop;
    prop.fromConfigFile(config_file);

    std::string pathToURDF = prop.find("urdf").asString();

    auto joints = prop.findGroup("joints");
    std::vector<std::string> jointList;
    for (int i=1; i < joints.size(); i++)
        jointList.push_back(joints.get(i).asString());

    
    auto ports = prop.findGroup("ports");
    std::vector<std::string> portList;
    for (int i=1; i < ports.size(); i++)
        portList.push_back(ports.get(i).asString());

    double sample_time = prop.find("sample_time").asFloat64();
    double numControlledJoints = prop.find("controlled_joints").asFloat64();

    yarp::os::Network yarp;

    GazeControl gazeController(pathToURDF, jointList, portList, numControlledJoints, sample_time);
    gazeController.set_cartesian_gains(0.01);
    gazeController.start();

    RPCServer server(&gazeController);

    if (!server.open()) {
        return 1;
    }

    while (true) {
        yInfo("Server running happily");
        yarp::os::Time::delay(10);
    }

    server.close();
    return 0;
}
