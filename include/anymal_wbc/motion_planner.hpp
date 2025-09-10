#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include "anymal_wbc/WbcReferenceMsg.h"
#include <boost/thread.hpp>

class MotionPlanner {
    public:
        MotionPlanner();
        void plannerLoop();
        void run();
        void input_callback(const geometry_msgs::Twist& msg);
        void load_parameters();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber input_sub_;
        ros::Publisher ref_pub_;

        Eigen::Vector3d velocity_command_ = Eigen::Vector3d(0.0, 0.0, 0.0); // hold the velocity reference
        double yaw_rate_command_ = 0.0; // hold the yaw rate reference
        double yaw_; // hold the current yaw angle
        anymal_wbc::WbcReferenceMsg ref_msg_;

        // Define control points of the body segment
        Eigen::Vector3d pi_body_;
        Eigen::Vector3d pf_body_;

        // Define control points of the foot spline
        Eigen::Vector3d pi_foot_LH_;
        Eigen::Vector3d pf_foot_LH_;
        Eigen::Vector3d pi_foot_RH_;
        Eigen::Vector3d pf_foot_RH_;
        Eigen::Vector3d pi_foot_LF_;
        Eigen::Vector3d pf_foot_LF_;
        Eigen::Vector3d pi_foot_RF_;
        Eigen::Vector3d pf_foot_RF_;

        struct {
            double step_length; // Length of the step
            double height_control_point;
            double x_offset;
            double y_offset;
            double step_duration; // Duration of each step phase
            double cycle_duration; // Duration of the entire cycle (4 steps)
            double body_height;
            double body_initial_velocity;
            double body_final_velocity;
            double dt;
        } params;        
};