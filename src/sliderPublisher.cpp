#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <QApplication>
#include <QTimer>
#include "sliderGui.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slider_publisher");
    QApplication app(argc, argv);

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/anymal/com_reference", 10);

    int num_sliders = 6; // Number of sliders
    SliderGUI gui(num_sliders);
    gui.show();

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        std_msgs::Float64MultiArray msg;
        msg.data = gui.getSliderValues();
        
        for (int i = 0 ; i < num_sliders ; i++)
        {
            msg.data[i] = msg.data[i]/1000.0;
        }

        pub.publish(msg);
        ros::spinOnce(); // Allow ROS to process incoming messages
    });
    timer.start(100); // Check the slider values every 100ms

    return app.exec();
}
