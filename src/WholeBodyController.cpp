#include "WholeBodyController.h"

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

    jointPos_.resize(model_.getNrOfDOFs());
    jointVel_.resize(model_.getNrOfDOFs());

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
    // make sure that the order of the joints from the msg is the same as the model
    if (firstJointStateCallback_)
    {
        for (int i = 0; i < model_.getNrOfDOFs(); i++)
        {
            int k = 0;
            while (jointStateMsg.name[k] != model_.getJointName(i))
            {
                k++;
            }
            jointIndex_[i] = k;
        }
    }
    firstJointStateCallback_ = false;

    for (int i = 0; i < model_.getNrOfDOFs(); i++)
    {    
        jointPos_(i) = jointStateMsg.position[jointIndex_[i]];
        jointVel_(i) = jointStateMsg.velocity[jointIndex_[i]];
    }
}


void WholeBodyController::updateState()
{
    kinDynComp_.setRobotState( T_world_base_, jointPos_, baseVel_, jointVel_, gravity_);
}


void WholeBodyController::run()
{
    //starts the control loop thread
	boost::thread ctrl_loop_t ( &WholeBodyController::controlLoop, this);     
	ros::spin();	    
}


void WholeBodyController::controlLoop()
{
    updateState();
    ros::Rate loopRate(1.0);
    while (ros::ok())
    {
        std::cout << "T_world_base_:\n" << T_world_base_ << "\n\n";
        std::cout << "jointPos_:\n" << jointPos_ << "\n\n";
        std::cout << "baseVel_:\n" << baseVel_ << "\n\n";
        std::cout << "jointVel_:\n" << jointVel_ << "\n\n";
        std::cout << "gravity_:\n" << gravity_ << "\n\n";
        loopRate.sleep();
        test();
    }
}


void WholeBodyController::test()
{
    /*
    Eigen::MatrixXd massMatrix(6+model_.getNrOfDOFs(), 6+model_.getNrOfDOFs());
    kinDynComp_.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(massMatrix));
    std::cout << "massMatrix:\n" << massMatrix << "\n\n";

    Eigen::Vector3d centerOfMass = iDynTree::toEigen(kinDynComp_.getCenterOfMassPosition());
    std::cout << "Center of mass:\n" << centerOfMass << "\n\n";

    Eigen::Vector3d p_bc = centerOfMass - T_world_base_.block<3,1>(0,3);
    std::cout << "p_bc:\n" << p_bc << "\n\n";

    Eigen::MatrixXd centerOfMassJacobian(3, 6+model_.getNrOfDOFs());
    kinDynComp_.getCenterOfMassJacobian(iDynTree::make_matrix_view(centerOfMassJacobian));
    std::cout << "centerOfMassJacobian:\n" << centerOfMassJacobian << "\n\n";

    Eigen::MatrixXd freeFloatingJacobian(6, 6+model_.getNrOfDOFs());
    kinDynComp_.getFrameFreeFloatingJacobian("base",iDynTree::make_matrix_view(freeFloatingJacobian));
    std::cout << "freeFloatingJacobian:\n" << freeFloatingJacobian << "\n\n";

    Eigen::MatrixXd J_bc = centerOfMassJacobian.block<3,12>(0,6) - freeFloatingJacobian.block<3,12>(0,6);
    std::cout << "J_bc:\n" << J_bc << "\n\n";


    Eigen::MatrixXd transformMatrixInv(6+model_.getNrOfDOFs() , 6+model_.getNrOfDOFs());
    transformMatrixInv.setZero();
    transformMatrixInv.block<3,3>(0,0).setIdentity();
    transformMatrixInv.block<3,3>(0,3) = skewOperator(p_bc);
    transformMatrixInv.block<3,12>(0,6) = -J_bc;
    transformMatrixInv.block<3,3>(3,3).setIdentity();
    transformMatrixInv.block<12,12>(6,6).setIdentity();
    std::cout << "transformMatrixInv:\n" << transformMatrixInv << "\n\n";

    Eigen::MatrixXd massMatrixCenterOfMass(6+model_.getNrOfDOFs() , 6+model_.getNrOfDOFs());
    massMatrixCenterOfMass.setZero();
    massMatrixCenterOfMass = transformMatrixInv.transpose() * massMatrix * transformMatrixInv;
    std::cout << "massMatrixCenterOfMass:\n" << massMatrixCenterOfMass << "\n\n";


    // state variables
    Eigen::Matrix4d T_world_base_1;
    Eigen::VectorXd jointPos_1;
    Eigen::Matrix<double,6,1> baseVel_1;
    Eigen::VectorXd jointVel_1 ;
    Eigen::Vector3d gravity_1;

    jointPos_1.resize(model_.getNrOfDOFs());
    jointVel_1.resize(model_.getNrOfDOFs());


    kinDynComp_.getRobotState(T_world_base_1,jointPos_1,baseVel_1,jointVel_1,gravity_1);
    std::cout << "T_world_base_1:\n" << T_world_base_1 << "\n\n";
    std::cout << "jointPos_1:\n" << jointPos_1 << "\n\n";
    std::cout << "baseVel_1:\n" << baseVel_1 << "\n\n";
    std::cout << "jointVel_1:\n" << jointVel_1 << "\n\n";
    std::cout << "gravity_1:\n" << gravity_1 << "\n\n";
    */

   	USING_NAMESPACE_QPOASES

	/* Setup data of first QP. */
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setup data of second QP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };


	/* Setting up QProblem object. */
	QProblem example( 2,1 );

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 10;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

	/* Get and print solution of first QP. */
	real_t xOpt[2];
	real_t yOpt[2+1];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
	
	/* Solve second QP. */
	nWSR = 10;
	example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

	/* Get and print solution of second QP. */
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

	example.printOptions();

}


Eigen::Matrix3d skewOperator( Eigen::Vector3d v)
{
    Eigen::Matrix3d S;
    S << 0.0 , -v(2) ,  v(1),
         v(2),  0.0  , -v(0),
        -v(1),  v(0) ,  0.0 ; 
    return S;
}