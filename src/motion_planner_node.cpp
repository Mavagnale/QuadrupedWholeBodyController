#include "anymal_wbc/motion_planner.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_planner_node");
    MotionPlanner motion_planner;
    motion_planner.run();
    return 0;
}