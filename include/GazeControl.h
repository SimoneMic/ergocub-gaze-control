/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
*/
#ifndef GAZE_CONTROL__H
#define GAZE_CONTROL__H

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>   
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#include "JointInterface.h"
#include "QPSolver.h"

class GazeControl: public yarp::os::PeriodicThread
{
    private:
        JointInterface* m_jointInterface;
        QPSolver* m_solver;

        yarp::os::BufferedPort<yarp::os::Bottle> m_debugPort;
        iDynTree::KinDynComputations m_computer;

        unsigned int m_numJoints;                     // Number of joints being controlled
        unsigned int m_numControlledJoints;
        // Utils
		iDynTree::Transform Eigen_to_iDynTree(const Eigen::Isometry3d &T);                  // Converts from Eigen to iDynTree
		Eigen::Isometry3d   iDynTree_to_Eigen(const iDynTree::Transform &T);                // Converts from iDynTree to Eigen

        // Main Control Loop
        Eigen::VectorXd m_redundantTask;

        // PositionControl
        bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum);
		Eigen::Matrix<double,2,1> track_cartesian_trajectory(const double &time);
		Eigen::VectorXd track_joint_trajectory(const double &time);
        Eigen::VectorXd qRef;                                                               // Reference joint position to send to motors
        bool threadInit();
		void threadRelease();

        // Joint control
		double m_kp = 1.0;                                                     // Default proportional gain
		double m_kd = 2*sqrt(this->m_kp);                                      // Theoretically optimal damping
		// std::vector<iDynTree::CubicSpline> jointTrajectory;
		
        enum ControlSpace {joint, cartesian} m_controlSpace;
        bool m_isFinished = false;                                             // For regulating control actions
		double m_startTime, m_endTime;                                         // For regulating the control loop
		double m_sample_time = 0.01;

        Eigen::Matrix<double,2,1> pose_error(const Eigen::Vector3d &desired);                             
		 
        bool m_motors_enabled;

        std::string m_pathToURDF;
        std::vector<std::string> m_jointList;
        std::vector<std::string> m_portList;

        bool m_initialized;

    protected:
        // Kinematics & dynamics
		Eigen::VectorXd m_q, m_qdot;                 // Joint positions and velocities
        double m_image_width = 640;
        double m_image_height = 480;
        // Intrinsic Matrix
        Eigen::Matrix<double,3, 3> m_C = (Eigen::MatrixXd(3, 3) << 570.3422241210938, 0.0, 319.5,    // Intrinsic parameters
                                                                 0.0, 570.3422241210938, 239.5,
                                                                 0.0, 0.0, 1.0).finished();
        Eigen::MatrixXd m_J_R;                                                                      // Camera Jacobian
        Eigen::Matrix<double,6,6> m_M = Eigen::MatrixXd::Zero(6,6);
        Eigen::Matrix<double,2,6> m_J_I = (Eigen::MatrixXd(2,6) << 1.0, 0.0, 0.0,  0.0, 1.0, 0.0,
                                                                 0.0, 1.0, 0.0, -1.0, 0.0, 0.0).finished();
        Eigen::MatrixXd m_J;
        Eigen::Isometry3d m_cameraPose;              // Camera pose
         
        // Eigen::Vector3d desiredGaze = (Eigen::Vector3d() << -0.092092 - 0.1, 0.001294, 0.320022).finished();  // icub
        Eigen::Vector3d m_desiredGaze = (Eigen::Vector3d() << 0.074927 + 0.5, -0.011469, 1.523281 - 0.9).finished(); // eCub
        Eigen::Matrix<double, 2, 2> m_K;      // Gain

	public:
		GazeControl();
        
        bool configure(yarp::os::ResourceFinder &rf);

		bool update_state();

        bool set_cartesian_gains(const double &proportional);

        bool set_joint_gains(const double &proportional, const double &derivative);
        
        void set_motor_actuation(const bool enabled);

		bool set_gaze(const Eigen::Vector3d& desiredGaze);

        void run() override;
        // double getPeriod() override;	
};

#endif