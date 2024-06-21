#include "WholeBodyController.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    WholeBodyController wbc;
    wbc.run();

}