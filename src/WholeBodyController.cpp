#include "WholeBodyController.h"

Eigen::Matrix3d skewOperator( Eigen::Vector3d v )
{
    Eigen::Matrix3d S;
    S << 0.0 , -v(2) ,  v(1),
         v(2),  0.0  , -v(0),
        -v(1),  v(0) ,  0.0 ; 
    return S;
}

WholeBodyController::WholeBodyController() : initStatus_(1) , firstJointStateCallback_(true)
{
    std::string modelFile = ros::package::getPath("anymal_wbc") + "/urdf/anymal.urdf";
    initStatus_ = mdlLoader_.loadModelFromFile(modelFile);
    if ( !initStatus_ )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
    }

    initStatus_ = kinDynComp_.loadRobotModel(mdlLoader_.model());
    if( !initStatus_ )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:"
                  << std::endl << mdlLoader_.model().toString() << std::endl;
    }

    model_ = kinDynComp_.model();

    jointPos_.resize(numberOfJoints);
    jointVel_.resize(numberOfJoints);

    floatingBaseStateSub_ = nh_.subscribe("/gazebo/model_states" , 0 , &WholeBodyController::floatingBaseStateCallback , this);
    jointStateSub_ = nh_.subscribe("/anymal/joint_states" , 0 , &WholeBodyController::jointStateCallback , this);

    // initialize robot state
    T_world_base_.setIdentity();
    jointPos_.setZero();
    jointVel_.setZero();
    baseVel_.setZero();
    gravity_ << 0.0 , 0.0 , 9.81;
}

WholeBodyController::~WholeBodyController()
{

}

void WholeBodyController::floatingBaseStateCallback(gazebo_msgs::ModelStates modelStateMsg)
{
    Eigen::Quaterniond floatingBaseOrientationQuat( modelStateMsg.pose[1].orientation.w,    //todo: substitute "[1]" with the runtime model index
                                                    modelStateMsg.pose[1].orientation.x, 
                                                    modelStateMsg.pose[1].orientation.y,
                                                    modelStateMsg.pose[1].orientation.z );
    T_world_base_.block<3,3>(0,0) = floatingBaseOrientationQuat.toRotationMatrix();

    Eigen::Vector3d floatingBasePosition( modelStateMsg.pose[1].position.x,
                                          modelStateMsg.pose[1].position.y,
                                          modelStateMsg.pose[1].position.z );
    T_world_base_.block<3,1>(0,3) = floatingBasePosition;

    Eigen::Vector3d floatingBaseAngVel( modelStateMsg.twist[1].angular.x,
                                        modelStateMsg.twist[1].angular.y,
                                        modelStateMsg.twist[1].angular.z );

    Eigen::Vector3d floatingBaseLinVel( modelStateMsg.twist[1].linear.x,
                                        modelStateMsg.twist[1].linear.y,
                                        modelStateMsg.twist[1].linear.z );

    baseVel_ << floatingBaseLinVel , floatingBaseAngVel;
}


void WholeBodyController::jointStateCallback(sensor_msgs::JointState jointStateMsg)
{
    // make sure that the order of the joints from the msg is the same as the model (LH-LF-RF-RH)
    if (firstJointStateCallback_)
    {
        for (int i = 0; i < numberOfJoints; i++)
        {
            int k = 0;
            while (jointStateMsg.name[k] != model_.getJointName(i))
            {
                k++;
            }
            jointIndex_[i] = k;
        }
        firstJointStateCallback_ = false;
    }

    for (int i = 0; i < numberOfJoints; i++)
    {    
        jointPos_(i) = jointStateMsg.position[jointIndex_[i]];
        jointVel_(i) = jointStateMsg.velocity[jointIndex_[i]];
    }
}


void WholeBodyController::updateState()
{
    kinDynComp_.setRobotState( T_world_base_, jointPos_, baseVel_, jointVel_, gravity_);

    Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> transformationMatrix = computeTransformationMatrix();

    Eigen::MatrixXd massMatrix(6+numberOfJoints, 6+numberOfJoints);
    kinDynComp_.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(massMatrix));
    centroidMassMatrix_ =  transformationMatrix.inverse().transpose() * massMatrix * transformationMatrix.inverse();

    centroidStanceJacobian_ = computeStanceJacobian() * transformationMatrix.inverse();
}


Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> WholeBodyController::computeTransformationMatrix()
{
    Eigen::Vector3d centerOfMassPosition = iDynTree::toEigen(kinDynComp_.getCenterOfMassPosition());
    Eigen::Vector3d basePosition = T_world_base_.block<3,1>(0,3);
    Eigen::Vector3d p_bc = centerOfMassPosition - basePosition;

    Eigen::Matrix<double,6,6> centroidToBaseAdjointMatrix;
    centroidToBaseAdjointMatrix << Eigen::Matrix3d::Identity() , skewOperator(p_bc) ,
                                   Eigen::Matrix3d::Zero()     , Eigen::Matrix3d::Identity();

    Eigen::MatrixXd massMatrix(6+numberOfJoints, 6+numberOfJoints);
    kinDynComp_.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(massMatrix));
    Eigen::Matrix<double,6,6> baseMassMatrix =  massMatrix.block<6,6>(0,0);
    Eigen::Matrix<double,6,numberOfJoints> baseJointsMassMatrix =  massMatrix.block<6,numberOfJoints>(0,6);

    Eigen::Matrix<double,6,6+numberOfJoints> selectionMatrix;
    selectionMatrix << Eigen::Matrix<double,6,6>::Identity() , Eigen::Matrix<double,6,numberOfJoints>::Zero();

    // Note: the iDynTree method getCenterOfMassJacobian only computes the positional part of the CoM jacobian,
    //       in order to also obtain the rotational part (assuming frame oriented as the world frame), the following computation is done:

    Eigen::Matrix<double,6,6+numberOfJoints> centerOfMassFullJacobian = 
                            centroidToBaseAdjointMatrix.inverse() * baseMassMatrix.inverse() * selectionMatrix * massMatrix;

    Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> transformationMatrix;
    transformationMatrix << centerOfMassFullJacobian , Eigen::Matrix<double,numberOfJoints,6>::Zero() , Eigen::Matrix<double,numberOfJoints,numberOfJoints>::Identity();

    return transformationMatrix; 
}

Eigen::Matrix<double,12,6+numberOfJoints> WholeBodyController::computeStanceJacobian()
{
    Eigen::MatrixXd fullStanceJacobian(12, 6+numberOfJoints);
    Eigen::MatrixXd footStanceJacobian(6, 6+numberOfJoints);
    kinDynComp_.getFrameFreeFloatingJacobian(kinDynComp_.getFrameIndex("LH_FOOT"),iDynTree::make_matrix_view(footStanceJacobian));
    fullStanceJacobian.block<3,6+numberOfJoints>(0,0) = footStanceJacobian.block<3,6+numberOfJoints>(0,0);
    kinDynComp_.getFrameFreeFloatingJacobian(kinDynComp_.getFrameIndex("LF_FOOT"),iDynTree::make_matrix_view(footStanceJacobian));
    fullStanceJacobian.block<3,6+numberOfJoints>(3,0) = footStanceJacobian.block<3,6+numberOfJoints>(0,0);
    kinDynComp_.getFrameFreeFloatingJacobian(kinDynComp_.getFrameIndex("RF_FOOT"),iDynTree::make_matrix_view(footStanceJacobian));
    fullStanceJacobian.block<3,6+numberOfJoints>(6,0) = footStanceJacobian.block<3,6+numberOfJoints>(0,0);
    kinDynComp_.getFrameFreeFloatingJacobian(kinDynComp_.getFrameIndex("RH_FOOT"),iDynTree::make_matrix_view(footStanceJacobian));
    fullStanceJacobian.block<3,6+numberOfJoints>(9,0) = footStanceJacobian.block<3,6+numberOfJoints>(0,0);

    return fullStanceJacobian;
}


void WholeBodyController::run()
{
    // starts the control loop thread
	boost::thread ctrl_loop_t ( &WholeBodyController::controlLoop, this);     
	ros::spin();	    
}


void WholeBodyController::controlLoop()
{
    ros::Rate loopRate(1.0);
    while (ros::ok())
    {
        updateState();
        std::cout << "T_world_base_:\n" << T_world_base_ << "\n\n";
        std::cout << "jointPos_:\n" << jointPos_ << "\n\n";
        std::cout << "baseVel_:\n" << baseVel_ << "\n\n";
        std::cout << "jointVel_:\n" << jointVel_ << "\n\n";
//        std::cout << "centroidStanceJacobian_:\n" << centroidStanceJacobian_ << "\n\n";
//        std::cout << "centroidMassMatrix_:_\n" << centroidMassMatrix_ << "\n\n";
        loopRate.sleep();
    }
}

void WholeBodyController::test()
{
}