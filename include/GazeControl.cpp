/*
 * SPDX-FileCopyrightText: 2023-2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "GazeControl.h"
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>

GazeControl::GazeControl():	m_initialized(false),
						 	m_sample_time(0.01),
						 	yarp::os::PeriodicThread(m_sample_time)		 	
{
    //--------------------Default Parameters---------------------------
	m_pathToURDF = "/usr/local/src/robot/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/ergoCubSN001/model.urdf";
	m_jointList.resize(4);
	m_jointList[0] = "neck_roll";
	m_jointList[1] = "neck_pitch";
	m_jointList[2] = "neck_yaw";
	m_jointList[3] = "camera_tilt";
	m_numControlledJoints = m_jointList.size();
	m_portList.resize(2);
	m_portList[0] = "/ergocubSim/head";
	m_portList[1] = "/ergocubSim/torso";

	this->m_solver = new QPSolver();
};

bool GazeControl::configure(yarp::os::ResourceFinder &rf)
{
	bool okCheck = rf.check("GAZE_CONTROL");
    if (okCheck)
    {
        yarp::os::Searchable &config_prop = rf.findGroup("GAZE_CONTROL");
        if (config_prop.check("urdf"))
        {
           m_pathToURDF = config_prop.find("urdf").asString();
        }
		else
		{
			yWarning() << "[GazeControl::configure] Unable to find parameter: urdf in config file. Using default one";
		}
		yInfo() << "[GazeControl::configure] Using current URDF path: " << m_pathToURDF;

		// Controlled Joint List
		if (config_prop.check("joint_list"))
        {
			auto tmp_bottle = config_prop.find("joint_list").asList();
			m_jointList.resize(tmp_bottle->size());
			for (size_t i = 0; i < tmp_bottle->size() - 1; ++i)
			{
				m_jointList[i] = tmp_bottle->get(i).asString();
				yDebug() << "[GazeControl::configure] Controlling joint: " << m_jointList[i];
			}
			m_numControlledJoints = m_jointList.size();
        }
		else
		{	// Using default ones
			yWarning() << "[GazeControl::configure] Unable to find parameter: joint_list in config file. Using default one";

			for (size_t i = 0; i < m_jointList.size() - 1; ++i)
			{
				yDebug() << "[GazeControl::configure] Controlling joint: " << m_jointList[i];
			}
		}
		yDebug() << "[GazeControl::configure] Number of controlled joints: " << m_numControlledJoints;
		

		// ControlBoard Port Name List
		if (config_prop.check("port_list"))
        {
			auto tmp_bottle = config_prop.find("port_list").asList();
			m_portList.resize(tmp_bottle->size());
			for (size_t i = 0; i < tmp_bottle->size() - 1; ++i)
			{
				m_portList[i] = tmp_bottle->get(i).asString();
				yDebug() << "[GazeControl::configure] Using port: " << m_portList[i];
			}
        }
		else
		{	// Using default ones
			yWarning() << "[GazeControl::configure] Unable to find parameter: port_list in config file. Using default one";

			for (size_t i = 0; i < m_portList.size() - 1; ++i)
			{
				yDebug() << "[GazeControl::configure] Using port: " << m_portList[i];
			}
		}

		// Debug Port Name
		if (config_prop.check("debug_port_name"))
		{
			this->m_debugPort.open(config_prop.find("debug_port_name").asString());
		}
		else
		{
			this->m_debugPort.open("/GazeController/debug:o");
		}
    }

	m_q = Eigen::VectorXd::Zero(this->m_numJoints);				// Set the size of the position vector
	m_qdot = Eigen::VectorXd::Zero(this->m_numJoints);			// Set the size of the velocity vector
	m_J_R = Eigen::MatrixXd::Zero(6, m_numControlledJoints);	// Set the size of the Jacobian matrix (3 are the torso joint)
	m_J = Eigen::MatrixXd::Zero(2, m_numControlledJoints);

	// Setup joint interface
	this->m_jointInterface = new JointInterface(m_jointList, m_portList);

	// Redundant Task
	m_redundantTask.resize(this->m_numControlledJoints);
	m_redundantTask.setZero();
	
	// Load URDF
    iDynTree::ModelLoader loader;
	
	if(! loader.loadReducedModelFromFile(m_pathToURDF, m_jointList, "urdf"))
	{
		yError() << "[GazeControl::configure] Could not load model from the path: " << m_pathToURDF;
		return false;
	}
	// Successfully loaded urdf
    iDynTree::Model robot_model = loader.model();

    // Now load the model in to the KinDynComputations class	    
	if(! this->m_computer.loadRobotModel(robot_model))
	{
		yError() << "[GazeControl::configure] Could not generate iDynTree::KinDynComputations object from the model: " << loader.model().toString();
		return false;
	}
	yInfo() << "[GazeControl::configure] Successfully created iDynTree model from: " << m_pathToURDF;

	while (! this->update_state()){
		std::this_thread::sleep_for(std::chrono::milliseconds(int(m_sample_time * 1000.0)));
	}
	this->qRef = this->m_q.head(this->m_numControlledJoints);    // Start from current joint position  

	m_initialized = true;
	yInfo() << "[GazeControl::configure] Successfully Configured!";
	return true;
}

bool GazeControl::update_state()
{
	if(this->m_jointInterface->read_encoders(this->m_q, this->m_qdot))
	{		
		// Put data in iDynTree class to compute in
		// (there is probably a smarter way but I keep getting errors otherwise)
		iDynTree::VectorDynSize tempPosition(this->m_numJoints);
		iDynTree::VectorDynSize tempVelocity(this->m_numJoints);

		for(int i = 0; i < this->m_numJoints; i++)
		{
			tempPosition(i) = this->m_q(i);
			tempVelocity(i) = this->m_qdot(i);
		}

		// Put them in to the iDynTree class to solve the kinematics and dynamics
		if(this->m_computer.setRobotState(tempPosition,                                       // Joint positions
		                                tempVelocity,                                       // Joint velocities
		                                iDynTree::Vector3(std::vector<double> {0.0, 0.0, -9.81}))) // Direction of gravity
		{
			// Get the camera Jacobian
			Eigen::MatrixXd temp(6,6+this->m_numJoints);                                  // Temporary storage
			
			this->m_computer.getFrameFreeFloatingJacobian("realsense_rgb_frame",temp);    // Compute camera Jacobian "realsense_rgb_frame"  "eyes_tilt_frame"
			this->m_J_R = temp.middleCols(6,this->m_numControlledJoints);                             // Remove floating base and torso
			
            // Update camera pose
			this->m_cameraPose  = iDynTree_to_Eigen(this->m_computer.getWorldTransform("realsense_rgb_frame"));  // realsense_rgb_frame  "eyes_tilt_frame"
			
			// Camera Rotation Matrix
			this->m_M.topLeftCorner(3, 3)     = this->m_cameraPose.rotation().transpose();
			this->m_M.bottomRightCorner(3, 3) = this->m_cameraPose.rotation().transpose();

			// Compute the visual servoing Jacobian
			this->m_J = this->m_J_I * this->m_M * this->m_J_R;
			
			return true;
		}
		else
		{
			yError() << "[GazeControl::update_state] Could not set state for the iDynTree::iKinDynComputations object";
			return false;
		}
	}
	else
	{
		yError() << "[GazeControl::update_state] Could not update state from the JointInterface class";	  
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Convert iDynTree::Transform to Eigen::Isometry3d                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d GazeControl::iDynTree_to_Eigen(const iDynTree::Transform &T)
{
	iDynTree::Position pos = T.getPosition();
	iDynTree::Vector4 quat = T.getRotation().asQuaternion();
	
	return Eigen::Translation3d(pos[0],pos[1],pos[2])*Eigen::Quaterniond(quat[0],quat[1],quat[2],quat[3]);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Move the gaze to the desired pose                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
// bool GazeControl::move_to_pose(const Eigen::Isometry3d &cameraPose,
//                                const double &time)
// {
// 	// Put them in to std::vector objects and pass onward
// 	std::vector<Eigen::Isometry3d> cameraPoses(1,cameraPose);
// 	std::vector<double> times(1,time);
	
// 	return move_to_poses(cameraPoses, times);                                           // Call full function
// }


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Move the gaze through multiple poses                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool GazeControl::set_gaze(const Eigen::Vector3d& desiredGaze)
{
	try
	{
		this->m_controlSpace = cartesian;  
		this->m_desiredGaze = desiredGaze;
		return true;
	}
	catch(std::exception &exception)
	{
		yError() << "[GazeControl::set_gaze] Unable to set new Cartesian trajectories: " << exception.what();
		return false;
	}
	return true;
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the error between a desired and actual pose                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,2,1> GazeControl::pose_error(const Eigen::Vector3d &desired_robot)
{
	Eigen::Matrix<double,2,1> error;                                                                // Value to be computed
	Eigen::Matrix<double,2, 1> actual = (Eigen::MatrixXd(2, 1) << this->m_image_width / 2, this->m_image_height / 2).finished();


	// Change the point frame from robot to camera
	Eigen::Matrix<double,3, 3> r = this->m_cameraPose.rotation().transpose();
	Eigen::Matrix<double,3, 1> t = this->m_cameraPose.translation();
	Eigen::Matrix<double,3, 1> desired_camera = r * (desired_robot - t);

	// Project the point to the camera plane
	Eigen::Matrix<double,2,1> uv = ((this->m_C * desired_camera) / desired_camera[2]).head(2);


	Eigen::Vector3d desiredSight = (this->m_desiredGaze - this->m_cameraPose.translation()).normalized();
	Eigen::Vector3d actualSight = this->m_cameraPose.rotation().col(2);

	// yarp::os::Bottle& debugOutput = this->debugPort.prepare();
	// debugOutput.clear();

	// yarp::os::Bottle& point1 = debugOutput.addList();
	// yarp::os::Bottle& point2 = debugOutput.addList();

	// point1.addFloat64(desiredSight(0));
	// point1.addFloat64(desiredSight(1));
	// point1.addFloat64(desiredSight(2));

	
	// point2.addFloat64(actualSight(0));
	// point2.addFloat64(actualSight(1));
	// point2.addFloat64(actualSight(2));

	// this->debugPort.write();

	error = uv - actual;

	return error;
}
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Set the Cartesian gains                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool GazeControl::set_cartesian_gains(const double &proportional)
{
	if(proportional < 0)
	{
		yError() << "[GazeControl::set_cartesian_gains] Gains must be positive, but the inputs was: " << proportional;
		return false;
	}
	else{
		this->m_K = proportional * Eigen::MatrixXd::Identity(2, 2);
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Set the joint gains                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool GazeControl::set_joint_gains(const double &proportional, const double &derivative)
{
	if(proportional < 0 or derivative < 0)
	{
		yError() << "[GazeControl::set_joint_gains] Gains must be positive, but the inputs were for the proportional: " << proportional
				<< " and " << derivative;         
		return false;
	}
	else
	{
		this->m_kp = proportional;
		this->m_kd = derivative;
		
		return true;
	}
}

void GazeControl::set_motor_actuation(const bool enabled)
{
	m_motors_enabled = enabled;
}

// double GazeControl::getPeriod()
// {
// 	return this->sample_time;
// }


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void GazeControl::run()
{
	if (! m_initialized)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(int(m_sample_time * 1000)));
		return;
	}
	

	this->m_solver->clear_last_solution();                                                        // In the QP solver
	update_state();                                                                             // Update kinematics & dynamics for new control loop
	
	double elapsedTime = yarp::os::Time::now() - this->m_startTime;                               // Time since activation of control loop
	
	if(this->m_controlSpace == joint)
	{
		// Eigen::VectorXd qd(this->numJoints);
		
		// for(int i = 0; i < this->numJoints; i++)
		// {
		// 	qd(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);
			
		// 	if(qd(i) < this->jointInterface->positionLimit[i][0])
		// 		qd(i) = this->jointInterface->positionLimit[i][0] + 0.001;              // Just above the lower limit
		// 	if(qd(i) > this->jointInterface->positionLimit[i][1])
		// 		qd(i) = this->jointInterface->positionLimit[i][1] - 0.001;              // Just below the upper limit
		// }
		
		// this->qRef = qd;                                                                    // Reference position for joint motors
	}
	else
	{
		Eigen::VectorXd dq(this->m_numControlledJoints);                                                        // We want to solve this
		Eigen::VectorXd q0(this->m_numControlledJoints);
		
		// Calculate instantaneous joint limits
		Eigen::VectorXd lowerBound(this->m_numControlledJoints), upperBound(this->m_numControlledJoints);
		for(int i = 0; i < this->m_numControlledJoints; i++)
		{
			double lower, upper;
			compute_joint_limits(lower,upper,i);
			
			lowerBound(i) = lower;
			upperBound(i) = upper;
			q0(i) = 0.5*(lower + upper);
		}

		Eigen::VectorXd dx = track_cartesian_trajectory(elapsedTime);                       // Get the desired Cartesian motion
		Eigen::MatrixXd W = Eigen::MatrixXd::Identity(this->m_J.cols(), this->m_J.cols()) * 0.1;
				
		try // to solve the joint motion
		{
			// for(int j=0;j<4;j++)
			// {
			// 	redundantTask(j) = 0.1*(0.5*(this->jointInterface->positionLimit[j][0] + this->jointInterface->positionLimit[j][1]) - this->q(j));
			// }
		    m_redundantTask(0) = (0.0 - this->m_q(0)) * 7;


			dq = this->m_solver->least_squares(m_redundantTask,
		                           			 W,
								             dx,                                                      
		                                     this->m_J,                                                 // Constraint matrix
		                                     lowerBound,
		                                     upperBound,
		                                     q0);                                                     // Start point
		}
		catch(const char* error_message)
		{
			yError() << "[GazeControl::run] Caught unexpected exception while solving the joint motion: " << error_message;
			dq.setZero();
		}

		yarp::os::Bottle& debugOutput = this->m_debugPort.prepare();
		debugOutput.clear();

		yarp::os::Bottle& point1 = debugOutput.addList();
		yarp::os::Bottle& point2 = debugOutput.addList();

		point1.addFloat64(this->m_q(0));
		point1.addFloat64(this->m_q(1));
		point1.addFloat64(this->m_q(2));
		point1.addFloat64(this->m_q(3));

		
		point2.addFloat64(this->qRef(0));
		point2.addFloat64(this->qRef(1));
		point2.addFloat64(this->qRef(2));
		point2.addFloat64(this->qRef(3));

		this->m_debugPort.write();

		this->qRef += dq * m_sample_time;
		
	}

	// Reference displacement error check: if a too wide reference is asked, an error is shown and joint commands are not sent
	bool error_check = false;	
	for(int i = 0; i < this->m_numControlledJoints; i++){
		if (!error_check && ((this->qRef[i] * 180.0 / M_PI ) - (this->m_q[i] * 180.0 / M_PI) > 10 )){
			yError("Requested joint motion for joint %i is greater than 10 degrees.", i);
			error_check = true;
		}
			
		if (!error_check && ((this->qRef[i] * 180.0 / M_PI ) - (this->m_q[i] * 180.0 / M_PI) < -10)){
			yError("Requested joint motion for joint %i is greater than 10 degrees.", i);
			error_check = true;
		}
	}

	if (!error_check && m_motors_enabled){
		this->m_jointInterface->send_joint_commands(this->qRef);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute istantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool GazeControl::compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum)
{
	// NOTE TO FUTURE SELF: Need to compute a limit on the step size dq 
	
	if(jointNum > this->m_numJoints)
	{
		yError() << "[GazeControl::compute_joint_limits] Range of joint indices is 0 to " << m_numJoints - 1 << " but you called for " << jointNum;
		return false;
	}
	else
	{

		lower = this->m_jointInterface->positionLimit[jointNum][0] - this->qRef[jointNum];
		upper = this->m_jointInterface->positionLimit[jointNum][1] - this->qRef[jointNum];
		
		if(lower >= upper)
		{
			yError() << "[GazeControl::compute_joint_limits] Lower limit " << lower << " is greater than upper limit " << upper;
			return false;
		}
		else	return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve a discrete time step for Cartesian control                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,2,1> GazeControl::track_cartesian_trajectory(const double &time)
{
	// NOTE TO FUTURE SELF:
	// There are no checks here to see if the trajectory is queried correctly.
	// This could cause problems later
	
	//this->cameraTrajectory.get_state(pose,vel,acc,time);                                        // Desired state for the left hand


	Eigen::Matrix<double,2,1> dx = this->m_K*pose_error(this->m_desiredGaze);                      // Feedforward + feedback on the left hand
	// std::cout << "Before clipping " << dx.transpose() << std::endl;q
	dx = dx.cwiseMin(2);
	// std::cout << dx.transpose() << std::endl;
	// this->rightTrajectory.get_state(pose,vel,acc,time);                                      // Desired state for the right hand
	// dx.tail(6) = this->dt*vel + this->K*pose_error(pose,this->rightPose);                    // Feedforward + feedback on the right hand

	return dx;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Solve the step size to track the joint trajectory                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
// Eigen::VectorXd GazeControl::track_joint_trajectory(const double &time)
// {
// 	Eigen::VectorXd dq(this->numJoints); dq.setZero();                                          // Value to be returned
	
// 	for(int i = 0; i < this->numJoints; i++) dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->q[i];
	
// 	return dq;
// }


bool GazeControl::threadInit()
{
	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void GazeControl::threadRelease()
{
	// send_joint_commands(this->q);                                                               // Maintain current joint positions
}
