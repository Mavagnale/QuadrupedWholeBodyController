#ifndef REFERENCE_PUBLISHER_H
#define REFERENCE_PUBLISHER_H

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/thread.hpp>

#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>


class ReferencePublisher
{
    public:
        ReferencePublisher();
        
        void run();

        void plannerLoop();

    private:
};

#endif