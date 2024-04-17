/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */
    ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                    Demonstration of bimanual grasping with the ergoCub robot                  //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

#include "GazeControl.h"


// These are used for setting the length of trajectories
double longTime =  5.0;
double shortTime = 2.0;

// These are reference points with respect to the robot
double graspWidth    = 0.15;
double graspDist     = 0.45;
double graspRest     = 0.35;
double torsoHeight   = 0.90;
double nominalHeight = torsoHeight + 0.35;
double graspHeight   = nominalHeight;
double torsoDist     = 0.40;

// These are used for creating a Payload object but not really important
double mass = 0.1;
Eigen::Matrix<double,3,3> inertia = (Eigen::MatrixXd(3,3) << 1e-06,   0.0,   0.0,
                                                               0.0, 1e-06,   0.0,
                                                               0.0,   0.0, 1e-06).finished();


int main(int argc, char *argv[])
{
	yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
	yarp::os::ResourceFinder rf;
	rf.configure(argc, argv);

	GazeControl gazeControl;  
	if (! gazeControl.configure(rf))
	{
		yError() << "[ergocub-gaze-control] Unable to configure gazeControl in main";
		return -1;
	}
	gazeControl.set_cartesian_gains(1);

	// Configure communication across the yarp network
	yarp::os::RpcServer port;                                                      // Create a port for sending / receiving info
	double f = 0.5;
	double A = 0.1;
	Eigen::Vector3d root_to_camera = Eigen::Vector3d(0.074927, -0.011469, 1.523281 - 0.9);

	bool okCheck = rf.check("GAZE_CONTROL");
    if (okCheck)
    {
		yarp::os::Searchable &config = rf.findGroup("GAZE_CONTROL");
		if (config.check("rpc_local_name"))
		{
			port.open(config.find("rpc_local_name").asString());
		}
		else	// default value
		{
			port.open("/GazeController/command");             // Open the port with the name '/command'
		}

		if (config.check("f"))
		{
			f = config.find("f").asFloat64();
		}
		if (config.check("A"))
		{
			A = config.find("A").asFloat64();
		}

		if (config.check("root_to_camera_x") && config.check("root_to_camera_y") && config.check("root_to_camera_z"))
		{
			Eigen::Vector3d root_to_camera = Eigen::Vector3d(config.find("root_to_camera_x").asFloat64(),
			 												 config.find("root_to_camera_y").asFloat64(),
															 config.find("root_to_camera_z").asFloat64());
		}
		else
		{
			yWarning() << "[ergocub-gaze-control] unable to find one or more config for root_to_camera. Using defaults.";
		}
	}

	//yarp::os::Bottle input;                                                        // Store information from the user input
	//yarp::os::Bottle output;                                                       // Store information to send to the user
	//std::string command;                                                           // Response message, command from user
		
	// Run the control loop
	bool active = true;
	double t = 0;
	//command = "look_at";

	while(active)
	{
		double y = std::cos(2 * M_PI * f * t) * A;
		double z = std::sin(2 * M_PI * f * t) * A;

		gazeControl.set_gaze(Eigen::Vector3d(0.5, 0.0, z) + root_to_camera);
		gazeControl.step();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		t += 0.001;
	}

	return 0;
}
