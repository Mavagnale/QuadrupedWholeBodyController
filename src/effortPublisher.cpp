#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "effort_publisher");
    ros::NodeHandle nh;

    // Publisher for the joint command
    ros::Publisher LF_HAA_pub = nh.advertise<std_msgs::Float64>("/anymal/LF_HAA_controller/command", 10);
    ros::Publisher LF_HFE_pub = nh.advertise<std_msgs::Float64>("/anymal/LF_HFE_controller/command", 10);
    ros::Publisher LF_KFE_pub = nh.advertise<std_msgs::Float64>("/anymal/LF_KFE_controller/command", 10);

    ros::Publisher LH_HAA_pub = nh.advertise<std_msgs::Float64>("/anymal/LH_HAA_controller/command", 10);
    ros::Publisher LH_HFE_pub = nh.advertise<std_msgs::Float64>("/anymal/LH_HFE_controller/command", 10);
    ros::Publisher LH_KFE_pub = nh.advertise<std_msgs::Float64>("/anymal/LH_KFE_controller/command", 10);

    ros::Publisher RF_HAA_pub = nh.advertise<std_msgs::Float64>("/anymal/RF_HAA_controller/command", 10);
    ros::Publisher RF_HFE_pub = nh.advertise<std_msgs::Float64>("/anymal/RF_HFE_controller/command", 10);
    ros::Publisher RF_KFE_pub = nh.advertise<std_msgs::Float64>("/anymal/RF_KFE_controller/command", 10);

    ros::Publisher RH_HAA_pub = nh.advertise<std_msgs::Float64>("/anymal/RH_HAA_controller/command", 10);
    ros::Publisher RH_HFE_pub = nh.advertise<std_msgs::Float64>("/anymal/RH_HFE_controller/command", 10);
    ros::Publisher RH_KFE_pub = nh.advertise<std_msgs::Float64>("/anymal/RH_KFE_controller/command", 10);
    // Set loop rate
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // Create a message and set the command value
        std_msgs::Float64 command_msg;


        command_msg.data = 0.0; 
        LF_HAA_pub.publish(command_msg);

        command_msg.data = 0.78; 
        LF_HFE_pub.publish(command_msg);

        command_msg.data = -0.78; 
        LF_KFE_pub.publish(command_msg);

        command_msg.data = 0.0; 
        LH_HAA_pub.publish(command_msg);

        command_msg.data = -0.78; 
        LH_HFE_pub.publish(command_msg);

        command_msg.data = 0.78; 
        LH_KFE_pub.publish(command_msg);

        command_msg.data = 0.0; 
        RF_HAA_pub.publish(command_msg);

        command_msg.data = 0.78; 
        RF_HFE_pub.publish(command_msg);

        command_msg.data = -0.78; 
        RF_KFE_pub.publish(command_msg);

        command_msg.data = 0.0; 
        RH_HAA_pub.publish(command_msg);

        command_msg.data = -0.78; 
        RH_HFE_pub.publish(command_msg);

        command_msg.data = 0.78; 
        RH_KFE_pub.publish(command_msg);


        // Log the published command
        ROS_INFO("Published joint command");

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
