#include "WholeBodyController.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wholeBodyControllerNode");
    WholeBodyController wbc;
    wbc.run();
    return 0;
}