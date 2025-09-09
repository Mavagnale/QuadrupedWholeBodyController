#ifndef WHOLE_BODY_CONTROLLER_HPP
#define WHOLE_BODY_CONTROLLER_HPP

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include "anymal_wbc/WbcReferenceMsg.h"

// iDynTree
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/EigenHelpers.h>

// qpOASES
#include <qpOASES.hpp>

// tf
#include <tf/transform_broadcaster.h>

const std::string modelName = "anymalModel";
const int numberOfJoints = 12;
const int numberOfLegs = 4;
const double gravityAcceleration = 9.81;
const int qpNumberOfVariables = 6 + numberOfJoints + 3*numberOfLegs + 3*numberOfLegs; // CoM accelerations + joints accelerations + ground reaction forces + slack variables
const int qpNumberOfConstraints = 6 + 3*numberOfLegs + 4*numberOfLegs + numberOfJoints + 6*numberOfLegs; // dynamics constraints + stance feet constraints + non-sliding constraints + torque limit constraints + swing legs constraints


class WholeBodyController
{
    public:
        WholeBodyController();
        ~WholeBodyController();
        
        void run();
    
		void floatingBaseStateCallback(gazebo_msgs::ModelStates modelStateMsg);
		void jointStateCallback(sensor_msgs::JointState jointStateMsg);
		void referenceCallback(anymal_wbc::WbcReferenceMsg refMsg);

        void updateState();

        void setInitialState();
        void loadParameters();

        void computeJointTorques();
        void computeDerivatives();
        void solveQP();


        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> computeTransformationMatrix();
        void computeJacobians(Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> & stanceJacobian,
                                   Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> & swingJacobian);
        Eigen::Vector<double,6+numberOfJoints> computeCoriolisBias(); 
        Eigen::Matrix<double,4*numberOfLegs,3*numberOfLegs> computeNonSlidingConstraints();
        Eigen::Vector<double,6> computeDesiredWrench();
        Eigen::Vector<double,3*numberOfLegs> computeCommandedAccelerationSwingLegs();
        Eigen::Vector<double,3*numberOfLegs> computeSwingFootPosition();
        Eigen::Vector<double,3*numberOfLegs> computeSwingFootVelocity();
        

        void resetRobotSimState();
        void controlLoop();

        void publishTransform();
    private:
        ros::NodeHandle nh_;
        
        ros::Publisher desiredGroundReactionForcesPub_;
        ros::Publisher jointTorquePub_;
        ros::Publisher centerOfMassPub_;
        ros::Publisher gazeboPub_;

        ros::Subscriber floatingBaseStateSub_;
        ros::Subscriber jointStateSub_;
        ros::Subscriber plannerReferenceSub_;

        bool initStatus_;
        bool firstJointStateCallback_;
        bool firstFloatingBaseStateCallback;
        bool firstControllerIteration_;

        // parameters
        struct {
            std::string modelName;
            double friction;
            double loopRate;
            double maxTorque;
            double kpValue;
            double kpValueZ;
            double kdValue;
            double kiValue;
            double kpSwingValue;
            double kdSwingValue;
            double slackWeight;
            std::vector<double> refData;
        } params;

        // iDynTree structs
        std::string modelName_;
        iDynTree::ModelLoader mdlLoader_;
        iDynTree::KinDynComputations kinDynComp_;
        iDynTree::Model model_;
        int jointIndex_[numberOfJoints];
        int modelIndex_;

        // state variables
        Eigen::Matrix4d T_world_base_;
        Eigen::Vector<double,numberOfJoints> jointPos_;
        Eigen::Matrix<double,6,1> baseVel_;
        Eigen::Vector<double,numberOfJoints> jointVel_;
        Eigen::Vector3d gravity_;

        Eigen::Vector<double,6> currentPose_;
        Eigen::Vector<double,6> centerOfMassVelocity_;

        // references
        Eigen::Vector<double,6> desiredPose_;
        Eigen::Vector<double,6> desiredCoMVelocity_;
        Eigen::Vector<double,6> desiredCoMAcceleration_;
        Eigen::Vector<double,3*numberOfLegs> desiredSwingLegsAcceleration_;
        Eigen::Vector<double,3*numberOfLegs> desiredSwingLegsVelocity_;
        Eigen::Vector<double,3*numberOfLegs> desiredSwingLegsPosition_;

        // model quantities
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> massMatrix_;
        Eigen::Matrix<double,6,6> massMatrixBase_;
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> transformationMatrix_;
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> centroidMassMatrix_;
        Eigen::Matrix<double,6,6> centroidMassMatrixBase_;
        Eigen::Matrix<double,numberOfJoints,numberOfJoints> centroidMassMatrixJoints_;
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> centroidStanceJacobian_;
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> centroidSwingJacobian_;
        Eigen::Matrix<double,3*numberOfLegs,6> centroidStanceJacobianCoM_;
        Eigen::Matrix<double,3*numberOfLegs,6> centroidSwingJacobianCoM_;
        Eigen::Matrix<double,3*numberOfLegs,numberOfJoints> centroidStanceJacobianJoints_;
        Eigen::Matrix<double,3*numberOfLegs,numberOfJoints> centroidSwingJacobianJoints_;
        Eigen::Vector<double,6+numberOfJoints> centroidGeneralizedBias_;
        Eigen::Vector3d centerOfMassPosition_;
        double totalMass_;

        // on-off foot contacts
        int footContacts_[numberOfLegs];

        // numerical derivation
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> oldTransformationMatrix_;
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> transformationMatrixDot_;
        Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> transformationMatrixDotInverse_;
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> oldCentroidStanceJacobian_;
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> centroidStanceJacobianDot_;
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> oldCentroidSwingJacobian_;
        Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> centroidSwingJacobianDot_;
        Eigen::Vector<double,6> integralError_;

        // quadratic problem
        qpOASES::SQProblem quadraticProblem_;
        Eigen::Vector<double,qpNumberOfVariables> qpSolution_;

        // tf
        tf::TransformBroadcaster broadcaster_;
        tf::Transform transform_;
};

#endif // WHOLE_BODY_CONTROLLER_HPP