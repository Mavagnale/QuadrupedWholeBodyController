#include "anymal_wbc/whole_body_controller.hpp"

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

WholeBodyController::WholeBodyController() : initStatus_(1) ,
                                             firstJointStateCallback_(true) , firstControllerIteration_(true) , firstFloatingBaseStateCallback(true) ,
                                             quadraticProblem_( qpNumberOfVariables , qpNumberOfConstraints ) 
{
    std::string modelFile = ros::package::getPath("anymal_wbc") + "/urdf/anymal.urdf";
    initStatus_ = mdlLoader_.loadModelFromFile(modelFile);
    if ( !initStatus_ )
    {
        ROS_ERROR_STREAM("KinDynComputationsWithEigen: impossible to load model from " << modelFile);
    }

    initStatus_ = kinDynComp_.loadRobotModel(mdlLoader_.model());
    if( !initStatus_ )
    {
        ROS_ERROR_STREAM("KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:"
                          << std::endl << mdlLoader_.model().toString());
    }

    model_ = kinDynComp_.model();

    jointTorquePub_ = nh_.advertise<std_msgs::Float64MultiArray>("/anymal/joint_effort_group_controller/command", 10);
    desiredGroundReactionForcesPub_ = nh_.advertise<std_msgs::Float64MultiArray>("/anymal/desired_ground_reaction_forces", 10);
    centerOfMassPub_ = nh_.advertise<geometry_msgs::Pose>("/anymal/com", 10);
    gazeboPub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 0);
    energyPub_ = nh_.advertise<std_msgs::Float64>("/anymal/tank_energy", 0);
    alphaPub_ = nh_.advertise<std_msgs::Float64>("/anymal/alpha", 0);

    floatingBaseStateSub_ = nh_.subscribe("/gazebo/model_states" , 0 , &WholeBodyController::floatingBaseStateCallback , this);
    jointStateSub_ = nh_.subscribe("/anymal/joint_states" , 0 , &WholeBodyController::jointStateCallback , this);
    plannerReferenceSub_ = nh_.subscribe("/anymal/reference" , 0 , &WholeBodyController::referenceCallback , this);
    baseForceSub_ = nh_.subscribe("/base_contact_sensor" , 0 , &WholeBodyController::baseForceCallback , this);

    // load parameters
    loadParameters();
    
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
    for (int i = 0 ; i < numberOfLegs ; i++)
    {
        footContacts_[i] = 1;
    }

    totalMass_ = model_.getTotalMass();

    T_world_base_.setIdentity();
    jointPos_.setZero();
    jointVel_.setZero();
    baseVel_.setZero();
    gravity_ << 0.0 , 0.0 , gravityAcceleration;

    // todo: get the actual first position of the robot joints instead of the hard-coded one
    jointPos_ << 0.0 , -0.4 , 0.8 , 0.0 , 0.4 , -0.8 , 0.0 , 0.4 , -0.8 , 0.0 , -0.4 , 0.8; 
    T_world_base_.block<3,1>(0,3) << 0.0 , 0.0 , 0.60;

    massMatrix_.setIdentity();
    transformationMatrix_.setIdentity();
    oldTransformationMatrix_.setIdentity();
    oldCentroidStanceJacobian_.setZero();
    oldCentroidSwingJacobian_.setZero();

    massMatrixBase_.setIdentity();
    centroidMassMatrix_.setIdentity();
    centroidMassMatrixBase_.setIdentity();
    centroidMassMatrixJoints_.setIdentity();
    centroidStanceJacobian_.setIdentity();
    centroidSwingJacobian_.setIdentity();
    centroidStanceJacobianCoM_.setIdentity();
    centroidSwingJacobianCoM_.setIdentity();
    centroidStanceJacobianJoints_.setIdentity();
    centroidSwingJacobianJoints_.setIdentity();

    centroidGeneralizedBias_.setIdentity();
    transformationMatrixDot_.setZero();
    centroidStanceJacobianDot_.setZero();
    centroidSwingJacobianDot_.setZero();
    centerOfMassPosition_.setZero();

    // set the initial reference pose
    for (int i = 0; i < 6; ++i) 
    {
        desiredPose_(i) = params.refData[i];
    }
    ROS_INFO_STREAM("Loaded reference pose: " << desiredPose_.transpose());

    desiredCoMVelocity_.setZero();
    desiredCoMAcceleration_.setZero();
    desiredSwingLegsAcceleration_.setZero();
    desiredSwingLegsVelocity_.setZero();
    desiredSwingLegsPosition_.setZero();    

    integralError_.setZero();

    tankEnergy_ = params.initialTankEnergy;
}

void WholeBodyController::loadParameters()
{   
    if (!nh_.getParam("/anymal_wbc/modelName", params.modelName))
        ROS_ERROR("Failed to get param 'modelName'");
    if (!nh_.getParam("/anymal_wbc/friction", params.friction))
        ROS_ERROR("Failed to get param 'friction'");
    if (!nh_.getParam("/anymal_wbc/kpValue", params.kpValue))
        ROS_ERROR("Failed to get param 'kpValue'");
    if (!nh_.getParam("/anymal_wbc/kpValueX", params.kpValueX))
        ROS_ERROR("Failed to get param 'kpValueX'");
    if (!nh_.getParam("/anymal_wbc/kpValueZ", params.kpValueZ))
        ROS_ERROR("Failed to get param 'kpValueZ'");
    if (!nh_.getParam("/anymal_wbc/kdValue", params.kdValue))
        ROS_ERROR("Failed to get param 'kdValue'");
    if (!nh_.getParam("/anymal_wbc/kiValue", params.kiValue))
        ROS_ERROR("Failed to get param 'kiValue'");
    if (!nh_.getParam("/anymal_wbc/kpSwingValue", params.kpSwingValue))
        ROS_ERROR("Failed to get param 'kpSwingValue'");
    if (!nh_.getParam("/anymal_wbc/kdSwingValue", params.kdSwingValue))
        ROS_ERROR("Failed to get param 'kdSwingValue'");
    if (!nh_.getParam("/anymal_wbc/loopRate", params.loopRate))
        ROS_ERROR("Failed to get param 'loopRate'");
    if (!nh_.getParam("/anymal_wbc/maxTorque", params.maxTorque))
        ROS_ERROR("Failed to get param 'maxTorque'");
    if (!nh_.getParam("/anymal_wbc/slackWeight", params.slackWeight))
        ROS_ERROR("Failed to get param 'slackWeight'");       
    if (!nh_.getParam("/anymal_wbc/initialReferencePose", params.refData)) 
        ROS_ERROR("Failed to get param 'initialReferencePose'");
    if (!nh_.getParam("/anymal_wbc/forceReference", params.forceReference)) 
        ROS_ERROR("Failed to get param 'forceReference'");
    if (!nh_.getParam("/anymal_wbc/kpForce", params.kpForce)) 
        ROS_ERROR("Failed to get param 'kpForce'");
    if (!nh_.getParam("/anymal_wbc/kiForce", params.kiForce)) 
        ROS_ERROR("Failed to get param 'kiForce'");
    if (!nh_.getParam("/anymal_wbc/enableEnergyTank", params.enableEnergyTank)) 
        ROS_ERROR("Failed to get param 'enableEnergyTank'");
    if (params.enableEnergyTank)
    {
        if (!nh_.getParam("/anymal_wbc/initialTankEnergy", params.initialTankEnergy)) 
            ROS_ERROR("Failed to get param 'initialTankEnergy'");
        tankEnergy_ = params.initialTankEnergy;
    }        
}

void WholeBodyController::referenceCallback(anymal_wbc::WbcReferenceMsg refMsg)
{
    for (int i = 0 ; i < 6 ; i++)
    {
        desiredPose_(i) = refMsg.desiredComPose.data[i];
    }
    for (int i = 0 ; i < 6 ; i++)
    {
        desiredCoMVelocity_(i) = refMsg.desiredComVelocity.data[i];
    }
    for (int i = 0 ; i < 6 ; i++)
    {
        desiredCoMAcceleration_(i) = refMsg.desiredComAcceleration.data[i];
    }
    for (int i = 0 ; i < 3*numberOfLegs ; i++)
    {
        desiredSwingLegsPosition_(i) = refMsg.desiredSwingLegsPosition.data[i];
    }
    for (int i = 0 ; i < 3*numberOfLegs ; i++)
    {
        desiredSwingLegsVelocity_(i) = refMsg.desiredSwingLegsVelocity.data[i];
    }
    for (int i = 0 ; i < 3*numberOfLegs ; i++)
    {
        desiredSwingLegsAcceleration_(i) = refMsg.desiredSwingLegsAcceleration.data[i];
    }
    for (int i = 0 ; i < numberOfLegs ; i++)
    {
        footContacts_[i] = refMsg.footContacts[i];
    }
}

void WholeBodyController::floatingBaseStateCallback(gazebo_msgs::ModelStates modelStateMsg)
{
    if (firstFloatingBaseStateCallback) 
    {
        modelIndex_ = 0;
        bool found = false;
        while (!found  && modelIndex_ < modelStateMsg.name.size()) 
        {
            if (modelStateMsg.name[modelIndex_] == modelName)
            {
                found = true;
                firstFloatingBaseStateCallback = true;
            } 
            else
            {
                modelIndex_++;
            }
        }
        firstFloatingBaseStateCallback = false;
    }
    else
    {
        Eigen::Quaterniond floatingBaseOrientationQuat( modelStateMsg.pose[modelIndex_].orientation.w,
                                                        modelStateMsg.pose[modelIndex_].orientation.x, 
                                                        modelStateMsg.pose[modelIndex_].orientation.y,
                                                        modelStateMsg.pose[modelIndex_].orientation.z );
        T_world_base_.block<3,3>(0,0) = floatingBaseOrientationQuat.toRotationMatrix();

        Eigen::Vector3d floatingBasePosition( modelStateMsg.pose[modelIndex_].position.x,
                                              modelStateMsg.pose[modelIndex_].position.y,
                                              modelStateMsg.pose[modelIndex_].position.z );
        T_world_base_.block<3,1>(0,3) = floatingBasePosition;

        Eigen::Vector3d floatingBaseAngVel( modelStateMsg.twist[modelIndex_].angular.x,
                                            modelStateMsg.twist[modelIndex_].angular.y,
                                            modelStateMsg.twist[modelIndex_].angular.z );

        Eigen::Vector3d floatingBaseLinVel( modelStateMsg.twist[modelIndex_].linear.x,
                                            modelStateMsg.twist[modelIndex_].linear.y,
                                            modelStateMsg.twist[modelIndex_].linear.z );

        baseVel_ << floatingBaseLinVel , floatingBaseAngVel;
    }
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

void WholeBodyController::baseForceCallback(gazebo_msgs::ContactsState contactMsg)
{
    if (contactMsg.states.size() > 0)
    {
        measuredBaseForce_ = contactMsg.states[0].wrenches[0].force.x;
    }
    else
    {
        measuredBaseForce_ = 0.0;
    }
}

void WholeBodyController::updateState()
{
    kinDynComp_.setRobotState( T_world_base_, jointPos_, baseVel_, jointVel_, gravity_);

    centerOfMassPosition_ = iDynTree::toEigen(kinDynComp_.getCenterOfMassPosition());
    centerOfMassVelocity_ << iDynTree::toEigen(kinDynComp_.getCenterOfMassVelocity()) , baseVel_.block<3,1>(3,0);
    Eigen::Matrix3d currentOrientation = T_world_base_.block<3,3>(0,0);
    Eigen::Vector<double,3> currentAttitude = eulAnglesRPY(currentOrientation);
    currentPose_ << centerOfMassPosition_ , currentAttitude;

    kinDynComp_.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(massMatrix_));
    massMatrixBase_ =  massMatrix_.block<6,6>(0,0);
    transformationMatrix_ = computeTransformationMatrix();

    centroidMassMatrix_ =  transformationMatrix_.inverse().transpose() * massMatrix_ * transformationMatrix_.inverse();
    centroidMassMatrixBase_ = centroidMassMatrix_.block<6,6>(0,0);
    centroidMassMatrixJoints_ = centroidMassMatrix_.block<numberOfJoints,numberOfJoints>(6,6);

    Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> stanceJacobian;
    Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> swingJacobian;
    computeJacobians( stanceJacobian , swingJacobian );

    centroidStanceJacobian_ = stanceJacobian * transformationMatrix_.inverse();
    centroidStanceJacobianCoM_ = centroidStanceJacobian_.block<3*numberOfLegs,6>(0,0);
    centroidStanceJacobianJoints_ =  centroidStanceJacobian_.block<3*numberOfLegs,numberOfJoints>(0,6);

    centroidSwingJacobian_ = swingJacobian * transformationMatrix_.inverse();
    centroidSwingJacobianCoM_ = centroidSwingJacobian_.block<3*numberOfLegs,6>(0,0);
    centroidSwingJacobianJoints_ =  centroidSwingJacobian_.block<3*numberOfLegs,numberOfJoints>(0,6);

    Eigen::Vector<double,6+numberOfJoints> generalizedBaseVel;
    generalizedBaseVel << baseVel_ , jointVel_;

    centroidGeneralizedBias_ = transformationMatrix_.inverse().transpose() * (computeCoriolisBias() +  massMatrix_ * transformationMatrixDotInverse_ * generalizedBaseVel); 

    computeDerivatives();
    transformationMatrixDotInverse_ = - transformationMatrix_.inverse() * transformationMatrixDot_ * transformationMatrix_.inverse();
}

Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> WholeBodyController::computeTransformationMatrix()
{
    Eigen::Vector3d basePosition = T_world_base_.block<3,1>(0,3);

    Eigen::Matrix<double,6,6> centroidToBaseAdjointMatrix;
    centroidToBaseAdjointMatrix << Eigen::Matrix3d::Identity() , skewOperator(centerOfMassPosition_ - basePosition) ,
                                   Eigen::Matrix3d::Zero()     , Eigen::Matrix3d::Identity();

    Eigen::Matrix<double,6,6+numberOfJoints> selectionMatrix;
    selectionMatrix << Eigen::Matrix<double,6,6>::Identity() , Eigen::Matrix<double,6,numberOfJoints>::Zero();

    // Note: the iDynTree method getCenterOfMassJacobian only computes the positional part of the CoM jacobian,
    //       in order to also obtain the rotational part (assuming centroidal frame oriented as the world frame), the following computation is done:

    Eigen::Matrix<double,6,6> centroidToBaseAdjointMatrixInverse = centroidToBaseAdjointMatrix;
    centroidToBaseAdjointMatrixInverse.block<3,3>(0,3) = -centroidToBaseAdjointMatrixInverse.block<3,3>(0,3);

    Eigen::Matrix<double,6,6+numberOfJoints> centerOfMassFullJacobian = 
                            centroidToBaseAdjointMatrixInverse * massMatrixBase_.inverse() * selectionMatrix * massMatrix_;

    Eigen::Matrix<double,6+numberOfJoints,6+numberOfJoints> transformationMatrix;
    transformationMatrix << centerOfMassFullJacobian , Eigen::Matrix<double,numberOfJoints,6>::Zero() , Eigen::Matrix<double,numberOfJoints,numberOfJoints>::Identity();

    return transformationMatrix;
}

void WholeBodyController::computeJacobians(Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> & stanceJacobian,
                                           Eigen::Matrix<double,3*numberOfLegs,6+numberOfJoints> & swingJacobian)
{
    Eigen::Matrix<double,6, 6+numberOfJoints> footJacobian;

    kinDynComp_.getFrameFreeFloatingJacobian( kinDynComp_.getFrameIndex("LH_FOOT") , iDynTree::make_matrix_view(footJacobian) );
    stanceJacobian.block<3,6+numberOfJoints>(0,0) = footJacobian.block<3,6+numberOfJoints>(0,0) * footContacts_[0];
    swingJacobian.block<3,6+numberOfJoints>(0,0) = footJacobian.block<3,6+numberOfJoints>(0,0) * !footContacts_[0];

    kinDynComp_.getFrameFreeFloatingJacobian( kinDynComp_.getFrameIndex("LF_FOOT") , iDynTree::make_matrix_view(footJacobian) );
    stanceJacobian.block<3,6+numberOfJoints>(3,0) = footJacobian.block<3,6+numberOfJoints>(0,0) * footContacts_[1];
    swingJacobian.block<3,6+numberOfJoints>(3,0) = footJacobian.block<3,6+numberOfJoints>(0,0) * !footContacts_[1];

    kinDynComp_.getFrameFreeFloatingJacobian( kinDynComp_.getFrameIndex("RF_FOOT") , iDynTree::make_matrix_view(footJacobian) );
    stanceJacobian.block<3,6+numberOfJoints>(6,0) = footJacobian.block<3,6+numberOfJoints>(0,0) * footContacts_[2];
    swingJacobian.block<3,6+numberOfJoints>(6,0) = footJacobian.block<3,6+numberOfJoints>(0,0) * !footContacts_[2];

    kinDynComp_.getFrameFreeFloatingJacobian( kinDynComp_.getFrameIndex("RH_FOOT") , iDynTree::make_matrix_view(footJacobian) );
    stanceJacobian.block<3,6+numberOfJoints>(9,0) = footJacobian.block<3,6+numberOfJoints>(0,0) * footContacts_[3];
    swingJacobian.block<3,6+numberOfJoints>(9,0) = footJacobian.block<3,6+numberOfJoints>(0,0) * !footContacts_[3];
}

Eigen::Vector<double,3*numberOfLegs> WholeBodyController::computeSwingFootPosition()
{
    Eigen::Vector<double,3*numberOfLegs> swingFootPosition;
    Eigen::Matrix4d swingFootTransform;

    kinDynComp_.getWorldTransform( kinDynComp_.getFrameIndex("LH_FOOT") , iDynTree::make_matrix_view(swingFootTransform) );
    swingFootPosition.block<3,1>(0,0) = swingFootTransform.block<3,1>(0,3);

    kinDynComp_.getWorldTransform( kinDynComp_.getFrameIndex("LF_FOOT") , iDynTree::make_matrix_view(swingFootTransform) );
    swingFootPosition.block<3,1>(3,0) = swingFootTransform.block<3,1>(0,3);

    kinDynComp_.getWorldTransform( kinDynComp_.getFrameIndex("RF_FOOT") , iDynTree::make_matrix_view(swingFootTransform) );
    swingFootPosition.block<3,1>(6,0) = swingFootTransform.block<3,1>(0,3);

    kinDynComp_.getWorldTransform( kinDynComp_.getFrameIndex("RH_FOOT") , iDynTree::make_matrix_view(swingFootTransform) );
    swingFootPosition.block<3,1>(9,0) = swingFootTransform.block<3,1>(0,3);

    return swingFootPosition;
}

Eigen::Vector<double,3*numberOfLegs> WholeBodyController::computeSwingFootVelocity()
{
    Eigen::Vector<double,3*numberOfLegs> swingFootVelocity;
    iDynTree::Twist swingFootTwist;

    swingFootTwist = kinDynComp_.getFrameVel( kinDynComp_.getFrameIndex("LH_FOOT") );
    swingFootVelocity.block<3,1>(0,0) = iDynTree::toEigen(swingFootTwist).block<3,1>(0,0);

    swingFootTwist = kinDynComp_.getFrameVel( kinDynComp_.getFrameIndex("LF_FOOT") );
    swingFootVelocity.block<3,1>(3,0) = iDynTree::toEigen(swingFootTwist).block<3,1>(0,0);

    swingFootTwist = kinDynComp_.getFrameVel( kinDynComp_.getFrameIndex("RF_FOOT") );
    swingFootVelocity.block<3,1>(6,0) = iDynTree::toEigen(swingFootTwist).block<3,1>(0,0);

    swingFootTwist = kinDynComp_.getFrameVel( kinDynComp_.getFrameIndex("RH_FOOT") );
    swingFootVelocity.block<3,1>(9,0) = iDynTree::toEigen(swingFootTwist).block<3,1>(0,0);

    return swingFootVelocity;
}

void WholeBodyController::computeDerivatives()
{
    const double timeStep = 1.0 / params.loopRate;

    transformationMatrixDot_ = (transformationMatrix_ - oldTransformationMatrix_) / timeStep ; 
    centroidStanceJacobianDot_ = (centroidStanceJacobian_ - oldCentroidStanceJacobian_) / timeStep ;
    centroidSwingJacobianDot_ = (centroidSwingJacobian_ - oldCentroidSwingJacobian_) / timeStep ;

    // todo: add filtering to the numerical derivative

    oldTransformationMatrix_ = transformationMatrix_;
    oldCentroidStanceJacobian_ = centroidStanceJacobian_;
    oldCentroidSwingJacobian_ = centroidSwingJacobian_;
}

Eigen::Matrix<double,4*numberOfLegs,3*numberOfLegs> WholeBodyController::computeNonSlidingConstraints()
{
    const Eigen::Vector3d tangentialVector1 = {1.0 , 0.0 , 0.0};
    const Eigen::Vector3d tangentialVector2 = {0.0 , 1.0 , 0.0};
    const Eigen::Vector3d normalVector = {0.0 , 0.0 , 1.0};

    Eigen::Matrix<double,4,3> D;
    D << (tangentialVector1 - params.friction*normalVector).transpose() ,
        -(tangentialVector1 + params.friction*normalVector).transpose() ,
         (tangentialVector2 - params.friction*normalVector).transpose() ,
        -(tangentialVector2 + params.friction*normalVector).transpose() ;

    Eigen::Matrix<double,4*numberOfLegs,3*numberOfLegs> Dfr;
    Dfr.setZero();
    Dfr.block<4,3>(0,0) = D * footContacts_[0];
    Dfr.block<4,3>(4,3) = D * footContacts_[1];
    Dfr.block<4,3>(8,6) = D * footContacts_[2];
    Dfr.block<4,3>(12,9) = D * footContacts_[3];

    return Dfr;
}

Eigen::Vector<double,6> WholeBodyController::computeDesiredWrench()
{
    Eigen::Vector<double,6> desiredWrench;
    Eigen::Matrix<double,6,6> kpMatrix = params.kpValue * Eigen::Matrix<double,6,6>::Identity();
    kpMatrix(0,0) = params.kpValueX;
    kpMatrix(2,2) = params.kpValueZ;
    Eigen::Matrix<double,6,6> kdMatrix = params.kdValue * Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,6> kiMatrix = params.kiValue * Eigen::Matrix<double,6,6>::Identity();

    Eigen::Vector<double,6> gravityWrench = { 0.0 , 0.0 , totalMass_*gravityAcceleration , 0.0 , 0.0 , 0.0 };

    desiredWrench = - kpMatrix * (currentPose_ - desiredPose_) 
                    - kdMatrix * (centerOfMassVelocity_ - desiredCoMVelocity_)
                    - kiMatrix * (integralError_)
                    + gravityWrench
                    + centroidMassMatrixBase_ * desiredCoMAcceleration_;

    integralError_ = integralError_ + (currentPose_ - desiredPose_)/params.loopRate; 

    desiredWrench(0) += energyTankUpdate();

    return desiredWrench;
}

double WholeBodyController::energyTankUpdate()
{
    double forceWrench = - params.kpForce * (params.forceReference - measuredBaseForce_) - params.kiForce * integralBaseForceError_;
    if (fabs(measuredBaseForce_) > 0.0)
    {
        integralBaseForceError_ = integralBaseForceError_ + (params.forceReference - measuredBaseForce_)/params.loopRate; 
    }

    if (params.enableEnergyTank)
    {
        double nominalPower = forceWrench * centerOfMassVelocity_(0);
        double dissipatedPower = 0.5 * params.kdValue * centerOfMassVelocity_(0) * centerOfMassVelocity_(0);
        double epsilon = 1e-3;
        double alpha = std::min(1.0 , tankEnergy_ / (fabs(nominalPower) / params.loopRate + epsilon) );
        double maxTankEnergy = 100.0;
        forceWrench = forceWrench * alpha;

        tankEnergy_ = tankEnergy_ + (- alpha * fabs(nominalPower) + dissipatedPower)/ params.loopRate;
        if (tankEnergy_ < 0.0)
        {
            tankEnergy_ = 0.0;
        }
        else if (tankEnergy_ > maxTankEnergy)
        {
            tankEnergy_ = maxTankEnergy;
        }
        
        std_msgs::Float64 energyMsg;
        energyMsg.data = tankEnergy_;
        energyPub_.publish(energyMsg);
        std_msgs::Float64 alphaMsg;
        alphaMsg.data = alpha;
        alphaPub_.publish(alphaMsg);
        
        ROS_INFO_STREAM("Tank energy: " << tankEnergy_ << ", alpha: " << alpha << ", measuredBaseForce_: " << measuredBaseForce_ );
    }
    else
    {
        ROS_INFO_STREAM("measuredBaseForce_: " << measuredBaseForce_ );
    }

    return forceWrench;
}


Eigen::Vector<double,3*numberOfLegs> WholeBodyController::computeCommandedAccelerationSwingLegs()
{
    Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs> kpSwingMatrix = params.kpSwingValue * Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Identity();
    Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs> kdSwingMatrix = params.kdSwingValue * Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Identity();

    Eigen::Vector<double,3*numberOfLegs> commandedAccelerationSwingLegs = desiredSwingLegsAcceleration_ +
                                                                          kdSwingMatrix * ( desiredSwingLegsVelocity_ - computeSwingFootVelocity() ) +
                                                                          kpSwingMatrix * ( desiredSwingLegsPosition_ - computeSwingFootPosition() ) ;

    for (int i = 0 ; i < numberOfLegs ; i++)
    {
        commandedAccelerationSwingLegs(3*i) *= !footContacts_[i];
        commandedAccelerationSwingLegs(3*i + 1) *= !footContacts_[i];
        commandedAccelerationSwingLegs(3*i + 2) *= !footContacts_[i];
    }

    return commandedAccelerationSwingLegs;
}

void WholeBodyController::solveQP()
{
    Eigen::Matrix<double,3*numberOfLegs, qpNumberOfVariables> groundReactionSelectionMatrix;
    groundReactionSelectionMatrix << Eigen::Matrix<double,3*numberOfLegs, 6 + numberOfJoints>::Zero() , Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Identity() , Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Zero() ;
    
    Eigen::Matrix<double,6,6> qpMatrixQ;
    Eigen::Matrix<double,qpNumberOfVariables,qpNumberOfVariables> qpMatrixR;
    qpMatrixQ.setIdentity();
    qpMatrixR.setIdentity();
    int slackVarIndex = 6 + numberOfJoints + 3*numberOfLegs;
    qpMatrixR.block<3*numberOfLegs, 3*numberOfLegs>(slackVarIndex,slackVarIndex) = params.slackWeight * Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Identity();

    // defined as Eigen::RowMajor because qpOASES expects arrays built by reading matrices by rows rather than columns
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

    qpMatrixA << centroidMassMatrixBase_                        , Eigen::Matrix<double,6,numberOfJoints>::Zero()              , -centroidStanceJacobianCoM_.transpose()                     , Eigen::Matrix<double,6,3*numberOfLegs>::Zero()                   ,
                 centroidStanceJacobianCoM_                     , centroidStanceJacobianJoints_                               , Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Zero() , Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Zero()      ,
                 Eigen::Matrix<double,4*numberOfLegs,6>::Zero() , Eigen::Matrix<double,4*numberOfLegs,numberOfJoints>::Zero() , Dfr                                                         , Eigen::Matrix<double,4*numberOfLegs,3*numberOfLegs>::Zero()      ,
                 Eigen::Matrix<double,numberOfJoints,6>::Zero() , centroidMassMatrixJoints_                                   , -centroidStanceJacobianJoints_.transpose()                  , Eigen::Matrix<double,numberOfJoints,3*numberOfLegs>::Zero()      , 
                 centroidSwingJacobianCoM_                      , centroidSwingJacobianJoints_                                , Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Zero() , -Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Identity() ,  
                 centroidSwingJacobianCoM_                      , centroidSwingJacobianJoints_                                , Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Zero() , Eigen::Matrix<double,3*numberOfLegs,3*numberOfLegs>::Identity()  ;  

    Eigen::Vector<double,numberOfJoints> currentJointVelocities = jointVel_;    // avoid race condition with the jointStateCallback    
    Eigen::Vector<double,6> gravityWrench;
    gravityWrench << 0 , 0 , totalMass_ * gravityAcceleration , 0 , 0 , 0;

    qpMatrixbUB <<  - gravityWrench,
                    - centroidStanceJacobianDot_.block<3*numberOfLegs,6>(0,0)*centerOfMassVelocity_ - centroidStanceJacobianDot_.block<3*numberOfLegs,numberOfJoints>(0,6)*currentJointVelocities ,
                    Eigen::Vector<double,4*numberOfLegs>::Zero(),
                    params.maxTorque * Eigen::Vector<double,numberOfJoints>::Ones() - centroidGeneralizedBias_.block<numberOfJoints,1>(6,0),
                    computeCommandedAccelerationSwingLegs() - centroidSwingJacobianDot_.block<3*numberOfLegs,6>(0,0)*centerOfMassVelocity_ - centroidSwingJacobianDot_.block<3*numberOfLegs,numberOfJoints>(0,6)*currentJointVelocities,
                    qpOASES::INFTY * Eigen::Vector<double,3*numberOfLegs>::Ones();

    qpMatrixbLB <<  - gravityWrench,
                    - centroidStanceJacobianDot_.block<3*numberOfLegs,6>(0,0)*centerOfMassVelocity_ - centroidStanceJacobianDot_.block<3*numberOfLegs,numberOfJoints>(0,6)*currentJointVelocities ,
                    -qpOASES::INFTY * Eigen::Vector<double,4*numberOfLegs>::Ones(),
                    - params.maxTorque * Eigen::Vector<double,numberOfJoints>::Ones() - centroidGeneralizedBias_.block<numberOfJoints,1>(6,0),
                    -qpOASES::INFTY * Eigen::Vector<double,3*numberOfLegs>::Ones(),
                    computeCommandedAccelerationSwingLegs() - centroidSwingJacobianDot_.block<3*numberOfLegs,6>(0,0)*centerOfMassVelocity_ - centroidSwingJacobianDot_.block<3*numberOfLegs,numberOfJoints>(0,6)*currentJointVelocities;

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

    std_msgs::Float64MultiArray desiredgrfMsg;
    for (int i = 0 ; i < 3*numberOfLegs ; i++)
    {
        desiredgrfMsg.data.push_back(desiredGroundReactionForces(i));
    } 
    desiredGroundReactionForcesPub_.publish(desiredgrfMsg);

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

void WholeBodyController::resetRobotSimState()
{
    ros::Rate rosRate(params.loopRate);

    double time = 0.0;
    const double deltaTime = 1.0 / params.loopRate;
    double resetTime = 0.5;
    double zOffset = 0.05;

    // needed to let the controllers load in gazebo
    while (time <= resetTime)
    {
        gazebo_msgs::ModelState stateMsg;
        stateMsg.model_name = params.modelName;
        stateMsg.reference_frame = "world";	
        geometry_msgs::Pose pose;
        // todo: use the base pose instead of the com pose
        pose.position.x = desiredPose_(0);
        pose.position.y = desiredPose_(1);
        pose.position.z = desiredPose_(2) + zOffset;        
        stateMsg.pose = pose;
        gazeboPub_.publish(stateMsg);

        setInitialState();
        rosRate.sleep();
        time = time + deltaTime;
    }
}

void WholeBodyController::publishTransform()
{
    transform_.setOrigin(tf::Vector3(T_world_base_(0,3), T_world_base_(1,3), T_world_base_(2,3)));
    Eigen::Matrix3d currentOrientation = T_world_base_.block<3,3>(0,0);
    Eigen::Vector<double,3> currentAttitude = eulAnglesRPY(currentOrientation);            
    tf::Quaternion q;
    q.setRPY(currentAttitude(0), currentAttitude(1), currentAttitude(2));  // roll, pitch, yaw
    transform_.setRotation(q);
    broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "map", "base"));
}

void WholeBodyController::controlLoop()
{
    ros::Rate rosRate(params.loopRate);

    resetRobotSimState();
    
    double time = 0.0;
    const double deltaTime = 1.0 / params.loopRate;
    long iteration = 0;

    while (ros::ok())
    {
        updateState();
        solveQP();
        computeJointTorques();

        // ROS_INFO_STREAM ("Elapsed time: " << time << " s");
        // ROS_INFO_STREAM ("Desired com position: " << desiredPose_.head<3>().transpose() );
        // ROS_INFO_STREAM ("Actual com position: " << centerOfMassPosition_.transpose() );
        // ROS_INFO_STREAM ("---------------------------");
        // ROS_INFO_STREAM("Measured base force: " << measuredBaseForce_);

        geometry_msgs::Pose comMsg;
        comMsg.position.x = centerOfMassPosition_(0);
        comMsg.position.y = centerOfMassPosition_(1);
        comMsg.position.z = centerOfMassPosition_(2);
        centerOfMassPub_.publish(comMsg);
        if (iteration % 10 == 0)
            publishTransform();
        rosRate.sleep();
        time = time + deltaTime;
    }
}

void WholeBodyController::run()
{
    // starts the control loop thread
	boost::thread ctrl_loop_t ( &WholeBodyController::controlLoop, this);     
	ros::spin();	    
}
