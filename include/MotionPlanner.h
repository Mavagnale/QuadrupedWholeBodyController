#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelState.h>
#include <boost/thread.hpp>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>

#include "towr/models/robot_model.h"
#include "towr/models/endeffector_mappings.h"
#include "towr/initialization/gait_generator.h"

const double loopRate = 500;

class MotionPlanner
{
    public:
        MotionPlanner();
        void plannerLoop();
        void publishReference();
        void publishGraphicReference();

    private:
        ros::NodeHandle nh_;
        ros::Publisher refPub_;
        ros::Publisher gazeboPub_;

        towr::NlpFormulation formulation_;

        Eigen::Vector<double,6> refPoseCoM_;
        Eigen::Vector<double,6> refVelocityCoM_; 
        Eigen::Vector<double,6> refAccelerationCoM_;

        Eigen::Vector<double,12> refPositionSwingLegs_;
        Eigen::Vector<double,12> refVelocitySwingLegs_;
        Eigen::Vector<double,12> refAccelerationSwingLegs_;

        int refContactLF_;
        int refContactRF_;
        int refContactLH_;
        int refContactRH_;

};

#endif //MOTION_PLANNER_H