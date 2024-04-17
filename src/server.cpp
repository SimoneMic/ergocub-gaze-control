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
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
	yarp::os::ResourceFinder rf;
	rf.configure(argc, argv);

	GazeControl gazeControl;  
	if (! gazeControl.configure(rf))
	{
		yError() << "[ergocub-gaze-control] [server.cpp] Unable to configure gazeControl in main";
		return -1;
	}
    
    gazeControl.set_cartesian_gains(0.01);
    gazeControl.start();

    RPCServer server(&gazeControl);

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
