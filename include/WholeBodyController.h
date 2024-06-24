#ifndef WHOLE_BODY_CONTROLLER_H
#define WHOLE_BODY_CONTROLLER_H

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>

// iDynTree
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/EigenHelpers.h>

// qpOASES
#include <qpOASES.hpp>

const int numberOfJoints = 12;

class WholeBodyController
{
    public:
        WholeBodyController();
        ~WholeBodyController();
        
        void run();
        void updateState();
        void test();
		void floatingBaseStateCallback(gazebo_msgs::ModelStates modelStateMsg);
		void jointStateCallback(sensor_msgs::JointState jointStateMsg);
        void controlLoop();


        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> computeTransformationMatrix();
        Eigen::Matrix<double,12,6+numberOfJoints> computeStanceJacobian();

    private:
        ros::NodeHandle nh_;
        
        ros::Publisher jointTorquePub_;

        ros::Subscriber floatingBaseStateSub_;
        ros::Subscriber jointStateSub_;

        bool initStatus_;
        bool firstJointStateCallback_;
        iDynTree::ModelLoader mdlLoader_;
        iDynTree::KinDynComputations kinDynComp_;
        iDynTree::Model model_;
        int jointIndex_[numberOfJoints];


        // state variables
        Eigen::Matrix4d T_world_base_;
        Eigen::VectorXd jointPos_;
        Eigen::Matrix<double,6,1> baseVel_;
        Eigen::VectorXd jointVel_;
        Eigen::Vector3d gravity_;

        // model matrices
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> centroidMassMatrix_;
        Eigen::Matrix<double,12,6+numberOfJoints> centroidStanceJacobian_;
};

#endif