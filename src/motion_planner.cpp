#include "anymal_wbc/motion_planner.hpp"


// Generate a simple cubic Bézier spline between four points
Eigen::Vector3d bezier_4_points(double s, const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                                 const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p;

    p = (1-s)*(1-s)*(1-s) * p0;
    p += 3 * (1-s)*(1-s) * s * p1;
    p += 3 * (1-s) * s*s * p2;
    p += s*s*s * p3;

    return p;
}

// Generate a cubic Bézier spline assuming that the control points are at fixed height above initial and final point
Eigen::Vector3d bezier(double s, const Eigen::Vector3d& pi, const Eigen::Vector3d& pf, double height_control_point) {
    Eigen::Vector3d vertical(0.0, 0.0, height_control_point);
    return bezier_4_points(s , pi, pi+vertical, pf+vertical, pf);
}

// Return a point of the segment pi to pf with normalized curvilinear abscissa s
Eigen::Vector3d segment(double s, const Eigen::Vector3d& pi, const Eigen::Vector3d& pf) {
    return pi + s*(pf-pi);
}

struct QuinticPolynomial {
    double a0, a1, a2, a3, a4, a5;

    double evaluate(double t) const {
        return a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
    }
};

/* Assumes s from 0 to 1, allows setting initial and final velocity (vi, vf), and takes as input the total period */
QuinticPolynomial generateQuinticPolynomial(double total_period, double vi = 0.0, double vf = 0.0) {
    QuinticPolynomial poly;
    double T = total_period;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    // Boundary conditions:
    // s(0) = 0, s(T) = 1
    // s'(0) = vi, s'(T) = vf
    // s''(0) = 0, s''(T) = 0

    poly.a0 = 0.0;
    poly.a1 = vi;
    poly.a2 = 0.0;
    poly.a3 = (10.0 - 4.0 * vf * T - 6.0 * vi * T) / T3;
    poly.a4 = (-15.0 + 7.0 * vf * T + 8.0 * vi * T) / T4;
    poly.a5 = (6.0 - 3.0 * vf * T - 3.0 * vi * T) / T5;

    return poly;
}

void MotionPlanner::load_parameters()
{   
    if (!nh_.getParam("/anymal_wbc/step_length", params.step_length))
        ROS_ERROR("Failed to get param 'step_length'");
    if (!nh_.getParam("/anymal_wbc/height_control_point", params.height_control_point))
        ROS_ERROR("Failed to get param 'height_control_point'");
    if (!nh_.getParam("/anymal_wbc/x_offset", params.x_offset))
        ROS_ERROR("Failed to get param 'x_offset'");
    if (!nh_.getParam("/anymal_wbc/y_offset", params.y_offset))
        ROS_ERROR("Failed to get param 'y_offset'");
    if (!nh_.getParam("/anymal_wbc/step_duration", params.step_duration))
        ROS_ERROR("Failed to get param 'step_duration'");
    if (!nh_.getParam("/anymal_wbc/body_height", params.body_height))
        ROS_ERROR("Failed to get param 'body_height'");
    if (!nh_.getParam("/anymal_wbc/body_initial_velocity", params.body_initial_velocity))
        ROS_ERROR("Failed to get param 'body_initial_velocity'");
    if (!nh_.getParam("/anymal_wbc/body_final_velocity", params.body_final_velocity))
        ROS_ERROR("Failed to get param 'body_final_velocity'");
    if (!nh_.getParam("/anymal_wbc/dt", params.dt))
        ROS_ERROR("Failed to get param 'dt'");
    params.cycle_duration = 4 * params.step_duration; // Update cycle duration based on step duration
}    

void MotionPlanner::input_callback(const geometry_msgs::Twist& msg) {
    velocity_command_(0) = msg.linear.x;
    velocity_command_(1) = msg.linear.y;
    yaw_rate_command_ = msg.angular.z; // Assuming angular.z is used for vertical velocity reference
    // ROS_INFO("Received velocity reference: [%f, %f, %f]", velocity_command_(0), velocity_command_(1), yaw_rate_command_);
}

MotionPlanner::MotionPlanner(){
    input_sub_ = nh_.subscribe("/cmd_vel", 1, &MotionPlanner::input_callback, this);
    ref_pub_ = nh_.advertise<anymal_wbc::WbcReferenceMsg>("/anymal/reference", 1);

    for (int i = 0 ; i < 6 ; ++i) {
        ref_msg_.desiredComPose.data.push_back(0.0);
    }
    ref_msg_.desiredComPose.data.at(2) = params.body_height;
    for (int i = 0 ; i < 6 ; ++i) {
        ref_msg_.desiredComVelocity.data.push_back(0.0);
    }
    for (int i = 0 ; i < 6 ; ++i) {
        ref_msg_.desiredComAcceleration.data.push_back(0.0);
    }
    for (int i = 0 ; i < 12 ; ++i) {
        ref_msg_.desiredSwingLegsPosition.data.push_back(0.0);
    }
    for (int i = 0 ; i < 12 ; ++i) {
        ref_msg_.desiredSwingLegsVelocity.data.push_back(0.0);
    }
    for (int i = 0 ; i < 12 ; ++i) {
        ref_msg_.desiredSwingLegsAcceleration.data.push_back(0.0);
    }
    for (int i = 0 ; i < 4 ; ++i) {
        ref_msg_.footContacts[i] = 1;
    }

    yaw_ = 0.0;

    // Define control points of the body segment
    pi_body_ << 0.0, 0.0, params.body_height;
    pf_body_ = pi_body_ + params.step_length * velocity_command_;

    Eigen::Vector3d RH_direction(0.0, -2*params.y_offset, 0.0);
    Eigen::Vector3d LF_direction(2*params.x_offset, 0.0, 0.0);
    Eigen::Vector3d RF_direction(2*params.x_offset, -2*params.y_offset, 0.0);

    // Define control points of the foot spline
    pi_foot_LH_ << pi_body_(0)-params.x_offset, pi_body_(1)+params.y_offset, 0.0;
    pi_foot_RH_ = pi_foot_LH_ + RH_direction;
    pi_foot_LF_ = pi_foot_LH_ + LF_direction;
    pi_foot_RF_ = pi_foot_LH_ + RF_direction;
    pf_foot_LH_ = pi_foot_LH_;
    pf_foot_RH_ = pi_foot_RH_;
    pf_foot_LF_ = pi_foot_LF_;
    pf_foot_RF_ = pi_foot_RF_;
}


void MotionPlanner::plannerLoop() {

    ros::Rate rate(1/params.dt);

    int step_phase = 0; // LH, RH, LF, RF
    int cycle_counter = 0; // 1 cycle is 4 step phases 
    double current_step_time = 0.0; // Time elapsed since start of the step
    double current_cycle_time = 0.0; // Time elapsed since start of cycle


    QuinticPolynomial poly_foot = generateQuinticPolynomial(params.step_duration, 0.0, 0.0);

    // At the first cycle, velocity should start from 0
    QuinticPolynomial poly_body_start = generateQuinticPolynomial(params.cycle_duration, 0.0, params.body_final_velocity);
    // Then, initial and final velocity should be the same for each cycle
    QuinticPolynomial poly_body_continue = generateQuinticPolynomial(params.cycle_duration, params.body_final_velocity, params.body_final_velocity); 



    while (ros::ok()) {

        if (velocity_command_ != Eigen::Vector3d::Zero() || yaw_rate_command_!=0) {  // Go into a 4 step phases loop if there is a velocity reference

            Eigen::Matrix3d rot_matrix;
            rot_matrix << cos(yaw_) , -sin(yaw_) , 0 , sin(yaw_) , cos(yaw_) , 0 , 0 , 0 , 1;
            Eigen::Vector3d velocity_command_rotated = rot_matrix*velocity_command_;

            double delta_yaw = yaw_rate_command_ * params.cycle_duration;
            Eigen::Matrix3d rot_matrix_delta;
            rot_matrix_delta << cos(delta_yaw) , -sin(delta_yaw) , 0 , sin(delta_yaw) , cos(delta_yaw) , 0 , 0 , 0 , 1;

            Eigen::Vector3d LH_vec(pi_foot_LH_(0)-pi_body_(0), pi_foot_LH_(1)-pi_body_(1), 0.0);
            Eigen::Vector3d RH_vec(pi_foot_RH_(0)-pi_body_(0), pi_foot_RH_(1)-pi_body_(1), 0.0);
            Eigen::Vector3d LF_vec(pi_foot_LF_(0)-pi_body_(0), pi_foot_LF_(1)-pi_body_(1), 0.0);
            Eigen::Vector3d RF_vec(pi_foot_RF_(0)-pi_body_(0), pi_foot_RF_(1)-pi_body_(1), 0.0);

            Eigen::Vector3d direction_new_LH = rot_matrix_delta*LH_vec - LH_vec;
            Eigen::Vector3d direction_new_RH = rot_matrix_delta*RH_vec - RH_vec;
            Eigen::Vector3d direction_new_LF = rot_matrix_delta*LF_vec - LF_vec;
            Eigen::Vector3d direction_new_RF = rot_matrix_delta*RF_vec - RF_vec;

            pf_foot_LH_ += velocity_command_rotated * params.step_length + direction_new_LH;
            pf_foot_RH_ += velocity_command_rotated * params.step_length + direction_new_RH;
            pf_foot_LF_ += velocity_command_rotated * params.step_length + direction_new_LF;
            pf_foot_RF_ += velocity_command_rotated * params.step_length + direction_new_RF;
            
            while (step_phase < 4) {    // Loop through the 4 steps
                if ( current_step_time < params.step_duration ) { // Still in the same step phase
                    double s_foot = poly_foot.evaluate(current_step_time);
                    Eigen::Vector3d point_bezier_LH = bezier(s_foot, pi_foot_LH_, pf_foot_LH_, params.height_control_point);
                    Eigen::Vector3d point_bezier_RH = bezier(s_foot, pi_foot_RH_, pf_foot_RH_, params.height_control_point);
                    Eigen::Vector3d point_bezier_LF = bezier(s_foot, pi_foot_LF_, pf_foot_LF_, params.height_control_point);
                    Eigen::Vector3d point_bezier_RF = bezier(s_foot, pi_foot_RF_, pf_foot_RF_, params.height_control_point);

                    switch (step_phase){
                        case 0: {
                            ref_msg_.desiredSwingLegsPosition.data.at(0) = point_bezier_LH(0);
                            ref_msg_.desiredSwingLegsPosition.data.at(1) = point_bezier_LH(1);
                            ref_msg_.desiredSwingLegsPosition.data.at(2) = point_bezier_LH(2);
                            ref_msg_.footContacts = {0,1,1,1};
                            break;
                        }
                        case 1: {
                            ref_msg_.desiredSwingLegsPosition.data.at(9) = point_bezier_RH(0);
                            ref_msg_.desiredSwingLegsPosition.data.at(10) = point_bezier_RH(1);
                            ref_msg_.desiredSwingLegsPosition.data.at(11) = point_bezier_RH(2);
                            ref_msg_.footContacts = {1,1,1,0};
                            break;
                        }
                        case 2: {
                            ref_msg_.desiredSwingLegsPosition.data.at(3) = point_bezier_LF(0);
                            ref_msg_.desiredSwingLegsPosition.data.at(4) = point_bezier_LF(1);
                            ref_msg_.desiredSwingLegsPosition.data.at(5) = point_bezier_LF(2);
                            ref_msg_.footContacts = {1,0,1,1};
                            break;
                        }
                        case 3: {
                            ref_msg_.desiredSwingLegsPosition.data.at(6) = point_bezier_RF(0);
                            ref_msg_.desiredSwingLegsPosition.data.at(7) = point_bezier_RF(1);
                            ref_msg_.desiredSwingLegsPosition.data.at(8) = point_bezier_RF(2);
                            ref_msg_.footContacts = {1,1,0,1};
                            break;
                        }
                        default: {
                            break;
                        }
                    }

                    double s_body = 0.0;
                    if (cycle_counter == 0) {
                        s_body = poly_body_start.evaluate(current_cycle_time);
                    } else {
                        s_body = poly_body_continue.evaluate(current_cycle_time);
                    }
                    Eigen::Vector3d point_segment_body = segment(s_body, pi_body_, pf_body_);

                    ref_msg_.desiredComPose.data.at(0) = point_segment_body(0);
                    ref_msg_.desiredComPose.data.at(1) = point_segment_body(1);
                    ref_msg_.desiredComPose.data.at(2) = point_segment_body(2);
                    ref_msg_.desiredComPose.data.at(3) = 0.0;
                    ref_msg_.desiredComPose.data.at(4) = 0.0;
                    ref_msg_.desiredComPose.data.at(5) = yaw_;
                    for (int i = 0 ; i < 6 ; ++i) {
                        ref_msg_.desiredComVelocity.data.at(i) = 0.0;
                    }
                    for (int i = 0 ; i < 6 ; ++i) {
                        ref_msg_.desiredComAcceleration.data.at(i) = 0.0;
                    }
                    ref_pub_.publish(ref_msg_);

                    yaw_ += yaw_rate_command_* params.dt;
                    current_step_time += params.dt;
                    current_cycle_time += params.dt;
                } else {
                    step_phase = step_phase + 1; // Increment step phase
                    // ROS_INFO("Phase changed to: %d",step_phase);
                    current_step_time = 0.0;
                }
                ros::spinOnce();
                rate.sleep();
            }
            if (step_phase == 4 ) {
                cycle_counter++;
                step_phase = 0; // Reset step phase for the next cycle
                current_cycle_time = 0.0; // Reset cycle time
                pi_body_ = pf_body_; // Update body position for the next cycle
                pf_body_ += velocity_command_rotated*params.step_length; // Move body in the direction of the velocity reference
                pi_foot_LH_= pf_foot_LH_; // Update foot spline position for the next cycle
                pi_foot_RH_= pf_foot_RH_;
                pi_foot_LF_= pf_foot_LF_;
                pi_foot_RF_= pf_foot_RF_;
                // ROS_INFO("Cycle completed: %d", cycle_counter);
            }
        }
        else {   // if zero cmd vel is received, stand still
            ref_msg_.footContacts = {1,1,1,1};
            ref_pub_.publish(ref_msg_);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void MotionPlanner::run()
{
    // starts the control loop thread
	boost::thread planner_loop_t ( &MotionPlanner::plannerLoop, this);     
	ros::spin();	    
}
