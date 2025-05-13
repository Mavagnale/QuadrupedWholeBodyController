#include "anymal_wbc/WholeBodyController.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "whole_body_controller_node");
    WholeBodyController wbc;
    wbc.run();
    return 0;
}