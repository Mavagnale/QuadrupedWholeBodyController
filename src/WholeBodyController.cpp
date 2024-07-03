#include "WholeBodyController.h"

Eigen::Matrix3d skewOperator( Eigen::Vector3d v )
{
    Eigen::Matrix3d S;
    S << 0.0 , -v(2) ,  v(1),
         v(2),  0.0  , -v(0),
        -v(1),  v(0) ,  0.0 ; 
    return S;
}


Eigen::Vector3d eulAnglesRPY( Eigen::Matrix3d R )
{
    double roll = atan2( R(2,1) , R(2,2) );    
    double pitch = atan2( -R(2,0) , sqrt( R(2,1)*R(2,1) + R(2,2)*R(2,2) ) );
    double yaw = atan2( R(1,0) , R(0,0) );

    Eigen::Vector3d attitude{ roll , pitch , yaw };
    return attitude;
}


WholeBodyController::WholeBodyController() : initStatus_(1) , firstJointStateCallback_(true) , firstControllerIteration_(true), 
                                             quadraticProblem_( qpNumberOfVariables , qpNumberOfConstraints )
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

    jointTorquePub_ = nh_.advertise<std_msgs::Float64MultiArray>("/anymal/joint_effort_group_controller/command", 10);

    floatingBaseStateSub_ = nh_.subscribe("/gazebo/model_states" , 0 , &WholeBodyController::floatingBaseStateCallback , this);
    jointStateSub_ = nh_.subscribe("/anymal/joint_states" , 0 , &WholeBodyController::jointStateCallback , this);
    centerOfMassReferenceSub_ = nh_.subscribe("/anymal/com_reference" , 0 , &WholeBodyController::centerOfMassReferenceCallback , this);

    // initialize robot state
    setInitialState();

    // initialize qp 
    qpSolution_.setZero();
}


WholeBodyController::~WholeBodyController()
{

}


void WholeBodyController::setInitialState()
{
    totalMass_ = model_.getTotalMass();

    T_world_base_.setIdentity();
    jointPos_.setZero();
    jointVel_.setZero();
    baseVel_.setZero();
    gravity_ << 0.0 , 0.0 , gravityAcceleration;

    // todo: get the actual first position of the robot instead of the hard-coded one
    jointPos_ << 0.03 , -0,4 , 0.8 , 0.03 , 0.4 , -0.8 , -0.03 , 0.4 , -0.8 , -0.03 , -0.4 , 0.8; 
    T_world_base_.block<3,1>(0,3) << 0.0 , 0.0 , 0.48;

    massMatrix_.setIdentity();
    transformationMatrix_.setIdentity();
    oldTransformationMatrix_.setIdentity();
    oldCentroidStanceJacobian_.setZero();

    baseMassMatrix_.setIdentity();
    centroidMassMatrix_.setIdentity();
    centroidMassMatrixJoints_.setIdentity();
    centroidStanceJacobian_.setIdentity();
    centroidStanceJacobianCoM_.setIdentity();
    centroidStanceJacobianJoints_.setIdentity();
    centroidGeneralizedBias_.setIdentity();
    transformationMatrixDot_.setZero();
    centroidStanceJacobianDot_.setZero();
    centerOfMassPosition_.setZero();

    desiredPose_ = { 0.0 , 0.0 , 0.48 , 0.0 , 0.0 , 0.0 };

}

void WholeBodyController::centerOfMassReferenceCallback(std_msgs::Float64MultiArray refMsg)
{
    std::cout <<"CoM Reference acquired\n";   
    for (int i = 0 ; i < 6 ; i++)
    {
        desiredPose_(i) = refMsg.data[i];
    }
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

    centerOfMassPosition_ = iDynTree::toEigen(kinDynComp_.getCenterOfMassPosition());

    kinDynComp_.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(massMatrix_));
    baseMassMatrix_ =  massMatrix_.block<6,6>(0,0);
    transformationMatrix_ = computeTransformationMatrix();

    centroidMassMatrix_ =  transformationMatrix_.inverse().transpose() * massMatrix_ * transformationMatrix_.inverse();
    centroidMassMatrixJoints_ = centroidMassMatrix_.block<numberOfJoints,numberOfJoints>(6,6);

    centroidStanceJacobian_ = computeStanceJacobian() * transformationMatrix_.inverse();
    centroidStanceJacobianCoM_ = centroidStanceJacobian_.block<3*numberOfLegs,6>(0,0);
    centroidStanceJacobianJoints_ =  centroidStanceJacobian_.block<3*numberOfLegs,numberOfJoints>(0,6);

    Eigen::Vector<double,6+numberOfJoints> generalizedBaseVel;
    generalizedBaseVel << baseVel_ , jointVel_;

    centroidGeneralizedBias_ = 
        transformationMatrix_.inverse().transpose() * computeCoriolisBias() +
        transformationMatrix_.inverse().transpose() * massMatrix_ * transformationMatrixDotInverse_ * generalizedBaseVel; 

    computeDerivatives();
    transformationMatrixDotInverse_ = - transformationMatrix_.inverse() * transformationMatrixDot_ * transformationMatrix_.inverse();
}


Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> WholeBodyController::computeTransformationMatrix()
{
    Eigen::Vector3d basePosition = T_world_base_.block<3,1>(0,3);

    Eigen::Matrix<double,6,6> centroidToBaseAdjointMatrix;
    centroidToBaseAdjointMatrix << Eigen::Matrix3d::Identity() , skewOperator(centerOfMassPosition_ - basePosition) ,
                                   Eigen::Matrix3d::Zero()     , Eigen::Matrix3d::Identity();

    Eigen::Matrix<double,6,numberOfJoints> baseJointsMassMatrix =  massMatrix_.block<6,numberOfJoints>(0,6);

    Eigen::Matrix<double,6,6+numberOfJoints> selectionMatrix;
    selectionMatrix << Eigen::Matrix<double,6,6>::Identity() , Eigen::Matrix<double,6,numberOfJoints>::Zero();

    // Note: the iDynTree method getCenterOfMassJacobian only computes the positional part of the CoM jacobian,
    //       in order to also obtain the rotational part (assuming centroidal frame oriented as the world frame), the following computation is done:

    Eigen::Matrix<double,6,6+numberOfJoints> centerOfMassFullJacobian = 
                            centroidToBaseAdjointMatrix.inverse() * baseMassMatrix_.inverse() * selectionMatrix * massMatrix_;

    Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> transformationMatrix;
    transformationMatrix << centerOfMassFullJacobian , Eigen::Matrix<double,numberOfJoints,6>::Zero() , Eigen::Matrix<double,numberOfJoints,numberOfJoints>::Identity();

    return transformationMatrix;
}

Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> WholeBodyController::computeStanceJacobian()
{
    Eigen::Matrix<double,3*numberOfLegs, 6+numberOfJoints> fullStanceJacobian;
    Eigen::Matrix<double,6, 6+numberOfJoints> footStanceJacobian;
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

void WholeBodyController::computeDerivatives()
{
    double timeStep = 1.0 / loopRate;

    transformationMatrixDot_ = (transformationMatrix_ - oldTransformationMatrix_) / timeStep ; 
    centroidStanceJacobianDot_ = (centroidStanceJacobian_ - oldCentroidStanceJacobian_) / timeStep ;

    // todo: add filtering to the numerical derivative

    oldTransformationMatrix_ = transformationMatrix_;
    oldCentroidStanceJacobian_ = centroidStanceJacobian_;
}

Eigen::Matrix<double,4*numberOfLegs,3*numberOfLegs> WholeBodyController::computeNonSlidingConstraints()
{
    const Eigen::Vector3d tangentialVector1 = {1.0 , 0.0 , 0.0};
    const Eigen::Vector3d tangentialVector2 = {0.0 , 1.0 , 0.0};
    const Eigen::Vector3d normalVector = {0.0 , 0.0 , 1.0};

    Eigen::Matrix<double,4,3> D;
    D << (tangentialVector1 - friction*normalVector).transpose() ,
        -(tangentialVector1 + friction*normalVector).transpose() ,
         (tangentialVector2 - friction*normalVector).transpose() ,
        -(tangentialVector2 + friction*normalVector).transpose() ;

    Eigen::Matrix<double,4*numberOfLegs,3*numberOfLegs> Dfr;
    Dfr.setZero();
    Dfr.block<4,3>(0,0) = D;
    Dfr.block<4,3>(4,3) = D;
    Dfr.block<4,3>(8,6) = D;
    Dfr.block<4,3>(12,9) = D;

    return Dfr;
}


Eigen::Vector<double,6> WholeBodyController::computeDesiredWrench()
{
    Eigen::Vector<double,6> desiredWrench;
    Eigen::Matrix<double,6,6> kpMatrix = kpValue * Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,6> kdMatrix = kdValue * Eigen::Matrix<double,6,6>::Identity();


    Eigen::Vector<double,6> desiredCoMVelocity = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };

    Eigen::Matrix3d currentOrientation = T_world_base_.block<3,3>(0,0);
    Eigen::Vector<double,3> currentAttitude = eulAnglesRPY(currentOrientation);    

    Eigen::Vector<double,6> currentPose;
    currentPose << centerOfMassPosition_ , currentAttitude;

    Eigen::Vector<double,6> currentCoMVelocity;
    currentCoMVelocity << iDynTree::toEigen(kinDynComp_.getCenterOfMassVelocity()) , baseVel_.block<3,1>(3,0);

    Eigen::Vector<double,6> gravityWrench = { 0.0 , 0.0 , totalMass_*gravityAcceleration , 0.0 , 0.0 , 0.0 };

    desiredWrench = - kpMatrix * (currentPose - desiredPose_) 
                    - kdMatrix * (currentCoMVelocity - desiredCoMVelocity)
                    + gravityWrench; //todo: add feedforward acceleration term here

    return desiredWrench;
}


void WholeBodyController::solveQP()
{
    Eigen::Matrix<double,3*numberOfLegs, qpNumberOfVariables> groundReactionSelectionMatrix;
    groundReactionSelectionMatrix << Eigen::Matrix<double,3*numberOfLegs, 6 + numberOfJoints>::Zero() , Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Identity();
    
    Eigen::Matrix<double,6,6> qpMatrixQ;
    Eigen::Matrix<double,qpNumberOfVariables,qpNumberOfVariables> qpMatrixR;
    qpMatrixQ.setIdentity();
    qpMatrixR.setIdentity();

    Eigen::Matrix<double,qpNumberOfVariables,qpNumberOfVariables, Eigen::RowMajor> qpMatrixH =
                        groundReactionSelectionMatrix.transpose() * centroidStanceJacobianCoM_ *
                        qpMatrixQ * centroidStanceJacobianCoM_.transpose() * groundReactionSelectionMatrix
                        + qpMatrixR;
    Eigen::Vector<double,qpNumberOfVariables> qpMatrixg =
                        - groundReactionSelectionMatrix.transpose() * centroidStanceJacobianCoM_ * qpMatrixQ * computeDesiredWrench();

    Eigen::Matrix<double, qpNumberOfConstraints, qpNumberOfVariables, Eigen::RowMajor> qpMatrixA;
    Eigen::Vector<double, qpNumberOfConstraints> qpMatrixbUB;
    Eigen::Vector<double, qpNumberOfConstraints> qpMatrixbLB;

    Eigen::Matrix<double,4*numberOfLegs,3*numberOfLegs> Dfr = computeNonSlidingConstraints();

    qpMatrixA << centroidMassMatrix_.block<6,6>(0,0) , Eigen::Matrix<double,6,numberOfJoints>::Zero() , -centroidStanceJacobianCoM_.transpose(),
                 centroidStanceJacobianCoM_          , centroidStanceJacobianJoints_                  , Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Zero(),
                 Eigen::Matrix<double,4*numberOfLegs,6+numberOfJoints>::Zero() , Dfr,
                 Eigen::Matrix<double,numberOfJoints,6>::Zero() , centroidMassMatrixJoints_ , -centroidStanceJacobianJoints_.transpose();

    Eigen::Vector<double,6> currentCoMVelocity;
    currentCoMVelocity << iDynTree::toEigen(kinDynComp_.getCenterOfMassVelocity()) , baseVel_.block<3,1>(3,0);

    qpMatrixbUB <<  0.0 ,
                    0.0 ,
                    - totalMass_ * gravityAcceleration ,
                    0.0 ,
                    0.0 ,
                    0.0 ,
                    - centroidStanceJacobianDot_.block<3*numberOfLegs,6>(0,0)*currentCoMVelocity - centroidStanceJacobianDot_.block<3*numberOfLegs,numberOfJoints>(0,6)*jointVel_ ,
                    Eigen::Vector<double,4*numberOfLegs>::Zero(),
                    maxTorque * Eigen::Vector<double,numberOfJoints>::Ones() - centroidGeneralizedBias_.block<numberOfJoints,1>(6,0);
                
    qpMatrixbLB = qpMatrixbUB;
    qpMatrixbLB.block<4*numberOfLegs,1>(6+3*numberOfLegs,0) = -qpOASES::INFTY * Eigen::Vector<double,4*numberOfLegs>::Ones();
    qpMatrixbLB.block<numberOfJoints,1>(6+3*numberOfLegs+4*numberOfLegs,0) = -maxTorque * Eigen::Vector<double,numberOfJoints>::Ones() - centroidGeneralizedBias_.block<numberOfJoints,1>(6,0);

	qpOASES::int_t nWSR = 100;
    qpOASES::Options myOptions;
    myOptions.setToReliable( );
    myOptions.printLevel = qpOASES::PL_LOW ;
    quadraticProblem_.setOptions( myOptions );

    if (firstControllerIteration_)
    {
        quadraticProblem_.init( qpMatrixH.data(), qpMatrixg.data(), qpMatrixA.data(),
                                nullptr, nullptr, qpMatrixbLB.data(), qpMatrixbUB.data(), nWSR, 0 );
        firstControllerIteration_ = false;
    }
    else
    {
        quadraticProblem_.hotstart( qpMatrixH.data(), qpMatrixg.data(), qpMatrixA.data(), 
                                    nullptr, nullptr, qpMatrixbLB.data(), qpMatrixbUB.data(), nWSR, 0 );
    }
    qpOASES::real_t xOpt[qpNumberOfVariables];
	quadraticProblem_.getPrimalSolution( xOpt );


    for (int i = 0 ; i < qpNumberOfVariables ; i++)
    {
        qpSolution_[i] = xOpt[i];
    }
}

Eigen::Vector<double,6+numberOfJoints> WholeBodyController::computeCoriolisBias()
{
    Eigen::Vector<double,6+numberOfJoints> generalizedBias;
    kinDynComp_.generalizedBiasForces(generalizedBias);
    Eigen::Vector<double,6+numberOfJoints> generalizedGravity;
    kinDynComp_.generalizedGravityForces(generalizedGravity);
    return (generalizedBias - generalizedGravity);
}


void WholeBodyController::computeJointTorques()
{
    Eigen::Vector<double,numberOfJoints> desiredJointsAccelerations = qpSolution_.block<numberOfJoints,1>(6,0);
    Eigen::Vector<double,numberOfJoints> desiredGroundReactionForces = qpSolution_.block<3*numberOfLegs,1>(6+numberOfJoints,0);

    Eigen::Vector<double,numberOfJoints> actuationTorque = 
                    centroidMassMatrixJoints_ * desiredJointsAccelerations +
                    centroidGeneralizedBias_.block<numberOfJoints,1>(6,0) 
                    - centroidStanceJacobianJoints_.transpose() * desiredGroundReactionForces;

    std_msgs::Float64MultiArray commandMsg;
    for (int i = 0 ; i < numberOfJoints ; i++)
    {
        commandMsg.data.push_back(actuationTorque(i));
    } 

    jointTorquePub_.publish(commandMsg);
}

void WholeBodyController::controlLoop()
{

    ros::Rate rosRate(loopRate);

    while (ros::ok())
    {
        updateState();
        solveQP();
        computeJointTorques();

        std::cout << "Desired com position:\n";
        std::cout << desiredPose_.block<3,1>(0,0) << "\n\n";
        std::cout << "Actual com position:\n";
        std::cout << centerOfMassPosition_ << "\n\n";
        std::cout << "---------------------------" << "\n\n";

        rosRate.sleep();
    }
}

void WholeBodyController::run()
{
    // starts the control loop thread
	boost::thread ctrl_loop_t ( &WholeBodyController::controlLoop, this);     
	ros::spin();	    
}
