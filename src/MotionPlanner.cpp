#include "anymal_wbc/MotionPlanner.hpp"

MotionPlanner::MotionPlanner()
{
    refPub_ = nh_.advertise<std_msgs::Float64MultiArray>("/anymal/reference", 0);
    gazeboPub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 0);

    // terrain
    formulation_.terrain_ = std::make_shared<towr::FlatGround>(0.0);

    // Kinematic limits and dynamic parameters of the robot
    formulation_.model_ = towr::RobotModel(towr::RobotModel::Anymal); // to add AnymalD

    // set the initial position of the robot
	formulation_.initial_base_.lin.at(towr::kPos) << 0.00, 0.00, 0.48;
    formulation_.initial_base_.ang.at(towr::kPos) << 0.00, 0.00, 0.00;
    formulation_.initial_base_.lin.at(towr::kVel) << 0.00, 0.00, 0.00;
    formulation_.initial_base_.ang.at(towr::kVel) << 0.00, 0.00, 0.00;

	auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();
    formulation_.initial_ee_W_ = nominal_stance_B;

    formulation_.initial_ee_W_.at(towr::LF) <<  0.503,  0.335, 0.000 ;
    formulation_.initial_ee_W_.at(towr::RF) <<  0.503, -0.335, 0.000 ;
    formulation_.initial_ee_W_.at(towr::LH) << -0.503,  0.335, 0.000 ;
    formulation_.initial_ee_W_.at(towr::RH) << -0.503, -0.335, 0.000 ;

    // define the desired goal state of the robot
    formulation_.final_base_.lin.at(towr::kPos) << 1.00, 0.00, 0.48;
    formulation_.final_base_.ang.at(towr::kPos) << 0.00, 0.00, 0.00;


    auto gait_gen_ = towr::GaitGenerator::MakeGaitGenerator(4);
    auto id_gait = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C1);

    gait_gen_->SetCombo(id_gait);

    formulation_.params_.ee_phase_durations_.clear();

    for (int ee = 0; ee < 4; ee++) {
        formulation_.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(3, ee));
        formulation_.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
    }

}

void MotionPlanner::plannerLoop()
{
    // Initialize the nonlinear-programming problem with the variables, constraints and costs.
    ifopt::Problem nlp;
    towr::SplineHolder solution;
    for (auto c : formulation_.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto c : formulation_.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto c : formulation_.GetCosts())
        nlp.AddCostSet(c);

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
    solver->SetOption("max_cpu_time", 20.0);

    std::cout << "Solving...\n";
    solver->Solve(nlp);
    std::cout << "\nSolving done, publishing...\n";

    double t = 0;
    ros::Rate rosRate(loopRate);
    while (t <= solution.base_linear_->GetTotalTime() + 1e-5) 
    {
        refPoseCoM_ << solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
        refVelocityCoM_ << solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
        refAccelerationCoM_ << solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();

        refPositionSwingLegs_ << solution.ee_motion_.at(towr::LH)->GetPoint(t).p(), 
                                 solution.ee_motion_.at(towr::LF)->GetPoint(t).p(), 
                                 solution.ee_motion_.at(towr::RF)->GetPoint(t).p(), 
                                 solution.ee_motion_.at(towr::RH)->GetPoint(t).p();

        refVelocitySwingLegs_ << solution.ee_motion_.at(towr::LH)->GetPoint(t).v(),
                                 solution.ee_motion_.at(towr::LF)->GetPoint(t).v(),
                                 solution.ee_motion_.at(towr::RF)->GetPoint(t).v(),
                                 solution.ee_motion_.at(towr::RH)->GetPoint(t).v();

        refAccelerationSwingLegs_ << solution.ee_motion_.at(towr::LH)->GetPoint(t).a(),
                                     solution.ee_motion_.at(towr::LF)->GetPoint(t).a(),
                                     solution.ee_motion_.at(towr::RF)->GetPoint(t).a(),
                                     solution.ee_motion_.at(towr::RH)->GetPoint(t).a();

        refContactLH_ = solution.phase_durations_.at(towr::LH)->IsContactPhase(t);
        refContactLF_ = solution.phase_durations_.at(towr::LF)->IsContactPhase(t);
        refContactRF_ = solution.phase_durations_.at(towr::RF)->IsContactPhase(t);
        refContactRH_ = solution.phase_durations_.at(towr::RH)->IsContactPhase(t);
        
        publishReference();
        publishGraphicReference();

        std::cout << "Time elapsed: " << t << std::endl;
        t = t  + 1/loopRate;
        
        rosRate.sleep();
    }

    while (ros::ok())
    {
        rosRate.sleep();
    }
    
}

void MotionPlanner::publishReference()
{
        std_msgs::Float64MultiArray msg;
        for (int i = 0; i < 6; i++) 
        {
            msg.data.push_back(refPoseCoM_(i));
        }
        for (int i = 0; i < 6; i++) 
        {
            msg.data.push_back(refVelocityCoM_(i));
        }
        for (int i = 0; i < 6; i++) 
        {
            msg.data.push_back(refAccelerationCoM_(i));
        }
        for (int i = 0; i < 12; i++)
        {
            msg.data.push_back(refPositionSwingLegs_(i));
        }
        for (int i = 0; i < 12; i++)
        {
            msg.data.push_back(refVelocitySwingLegs_(i));
        }
        for (int i = 0; i < 12; i++)
        {
            msg.data.push_back(refAccelerationSwingLegs_(i));
        }
        msg.data.push_back(refContactLH_);
        msg.data.push_back(refContactLF_);
        msg.data.push_back(refContactRF_);
        msg.data.push_back(refContactRH_);

        refPub_.publish(msg);
}

void MotionPlanner::publishGraphicReference()
{
        gazebo_msgs::ModelState stateMsg;
        stateMsg.model_name = "body_reference";
        stateMsg.reference_frame = "world";	
        geometry_msgs::Pose pose;
        
        pose.position.x = refPoseCoM_(0);
        pose.position.y = refPoseCoM_(1);
        pose.position.z = refPoseCoM_(2);

        Eigen::Quaterniond refPoseQuat;
        refPoseQuat = Eigen::AngleAxisd(refPoseCoM_(3), Eigen::Vector3d::UnitX()) 
                   * Eigen::AngleAxisd(refPoseCoM_(4), Eigen::Vector3d::UnitY()) 
                   * Eigen::AngleAxisd(refPoseCoM_(5), Eigen::Vector3d::UnitZ());

        pose.orientation.x = refPoseQuat.x();
        pose.orientation.y = refPoseQuat.y();
        pose.orientation.z = refPoseQuat.z();
        pose.orientation.w = refPoseQuat.w();

        stateMsg.pose = pose;
        gazeboPub_.publish(stateMsg);

        stateMsg.model_name = "LH_reference";
        pose.position.x = refPositionSwingLegs_(0);
        pose.position.y = refPositionSwingLegs_(1);
        pose.position.z = refPositionSwingLegs_(2);        
        stateMsg.pose = pose;
        gazeboPub_.publish(stateMsg);

        stateMsg.model_name = "LF_reference";
        pose.position.x = refPositionSwingLegs_(3);
        pose.position.y = refPositionSwingLegs_(4);
        pose.position.z = refPositionSwingLegs_(5);        
        stateMsg.pose = pose;
        gazeboPub_.publish(stateMsg);

        stateMsg.model_name = "RF_reference";
        pose.position.x = refPositionSwingLegs_(6);
        pose.position.y = refPositionSwingLegs_(7);
        pose.position.z = refPositionSwingLegs_(8);        
        stateMsg.pose = pose;
        gazeboPub_.publish(stateMsg);

        stateMsg.model_name = "RH_reference";
        pose.position.x = refPositionSwingLegs_(9);
        pose.position.y = refPositionSwingLegs_(10);
        pose.position.z = refPositionSwingLegs_(11);        
        stateMsg.pose = pose;
        gazeboPub_.publish(stateMsg);
}


int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;

    MotionPlanner motPlanner;

    motPlanner.plannerLoop();

    return 0;
}