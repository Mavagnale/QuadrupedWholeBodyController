#ifndef WHOLE_BODY_CONTROLLER_H
#define WHOLE_BODY_CONTROLLER_H

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <Eigen/Core>

// iDynTree
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/EigenHelpers.h>

// qpOASES
#include <qpOASES.hpp>

const int numberOfJoints = 12;
const int numberOfLegs = 4;
const double gravityAcceleration = 9.81;
const int qpNumberOfVariables = 6 + numberOfJoints + 3*numberOfLegs;
const int qpNumberOfConstraints = 6 + 3*numberOfLegs;
const double loopRate = 500;

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
		void centerOfMassReferenceCallback(std_msgs::Float64MultiArray refMsg);
        void setInitialState();
        void computeJointTorques();
        void computeDerivatives();
        void controlLoop();


        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> computeTransformationMatrix();
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> computeStanceJacobian();
        void solveQP();

    private:
        ros::NodeHandle nh_;
        
        ros::Publisher jointTorquePub_;

        ros::Subscriber floatingBaseStateSub_;
        ros::Subscriber jointStateSub_;
        ros::Subscriber centerOfMassReferenceSub_;

        bool initStatus_;
        bool firstJointStateCallback_;
        bool firstControllerIteration_;

        // iDynTree structs
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

        // references
        Eigen::Vector<double,6> desiredPose_;

        // model matrices
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> massMatrix_;
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> transformationMatrix_;
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> centroidMassMatrix_;
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> centroidStanceJacobian_;
        Eigen::Matrix<double,3*numberOfLegs,6> centroidStanceJacobianCoM_;
        Eigen::Matrix<double,3*numberOfLegs,numberOfJoints> centroidStanceJacobianJoints_;

        // numerical derivation
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> oldTransformationMatrix_;
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> transformationMatrixDot_;
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> oldCentroidStanceJacobian_;
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> centroidStanceJacobianDot_;

        // quadratic problem
        qpOASES::SQProblem quadraticProblem_;
        Eigen::Vector<double,6+numberOfJoints+3*numberOfLegs> qpSoln_;
};

#endif