#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "anymal_wbc/WbcReferenceMsg.h"
#include <Eigen/Dense>

visualization_msgs::Marker com_marker_msg;
visualization_msgs::Marker lh_marker_msg;
visualization_msgs::Marker lf_marker_msg;
visualization_msgs::Marker rf_marker_msg;
visualization_msgs::Marker rh_marker_msg;

visualization_msgs::Marker draw_point(const double px, const double py, const double pz) {
    visualization_msgs::Marker point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.ns = "planner";
    point.id = 0;
    point.type = visualization_msgs::Marker::SPHERE;
    point.action = visualization_msgs::Marker::ADD;

    point.scale.x = 0.03;  // Line width
    point.scale.y = 0.03;  // Line width
    point.scale.z = 0.03;  // Line width

    point.color.r = 1.0;
    point.color.g = 0.0;
    point.color.b = 0.0;
    point.color.a = 1.0;

    point.pose.position.x = px;
    point.pose.position.y = py;
    point.pose.position.z = pz;

    point.pose.orientation.w = 1.0;
    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    return point;
}

void set_position(visualization_msgs::Marker& point, const double px, const double py, const double pz){
    point.pose.position.x = px;
    point.pose.position.y = py;
    point.pose.position.z = pz;
}

visualization_msgs::Marker draw_rectangle( double px, double py, double pz, double yaw){
    visualization_msgs::Marker rectangle;
    rectangle.header.frame_id = "map";
    rectangle.header.stamp = ros::Time::now();
    rectangle.ns = "planner";
    rectangle.id = 0;
    rectangle.type = visualization_msgs::Marker::CUBE;
    rectangle.action = visualization_msgs::Marker::ADD;

    rectangle.scale.x = 0.55;  // Line width
    rectangle.scale.y = 0.35;  // Line width
    rectangle.scale.z = 0.05;  // Line width

    rectangle.color.r = 0.0;
    rectangle.color.g = 0.0;
    rectangle.color.b = 1.0;
    rectangle.color.a = 1.0;

    rectangle.pose.position.x = px;
    rectangle.pose.position.y = py;
    rectangle.pose.position.z = pz;

    rectangle.pose.orientation.w = cos(yaw/2);
    rectangle.pose.orientation.x = 0.0;
    rectangle.pose.orientation.y = 0.0;
    rectangle.pose.orientation.z = sin(yaw/2);

    return rectangle;
}

void input_callback(const anymal_wbc::WbcReferenceMsg& ref_msg) {
    com_marker_msg = draw_rectangle(ref_msg.desiredComPose.data[0], ref_msg.desiredComPose.data[1], ref_msg.desiredComPose.data[2], ref_msg.desiredComPose.data[5]);
    lh_marker_msg = draw_point(ref_msg.desiredSwingLegsPosition.data[0], ref_msg.desiredSwingLegsPosition.data[1], ref_msg.desiredSwingLegsPosition.data[2]);
    lf_marker_msg = draw_point(ref_msg.desiredSwingLegsPosition.data[3], ref_msg.desiredSwingLegsPosition.data[4], ref_msg.desiredSwingLegsPosition.data[5]);
    rf_marker_msg = draw_point(ref_msg.desiredSwingLegsPosition.data[6], ref_msg.desiredSwingLegsPosition.data[7], ref_msg.desiredSwingLegsPosition.data[8]);
    rh_marker_msg = draw_point(ref_msg.desiredSwingLegsPosition.data[9], ref_msg.desiredSwingLegsPosition.data[10], ref_msg.desiredSwingLegsPosition.data[11]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_publisher");
    ros::NodeHandle nh;

    ros::Subscriber ref_sub = nh.subscribe("anymal/reference", 1, input_callback);

    ros::Publisher body_pub = nh.advertise<visualization_msgs::Marker>("/body_marker", 1);
    ros::Publisher LH_pub = nh.advertise<visualization_msgs::Marker>("/LH_marker", 1);
    ros::Publisher LF_pub = nh.advertise<visualization_msgs::Marker>("/LF_marker", 1);
    ros::Publisher RH_pub = nh.advertise<visualization_msgs::Marker>("/RH_marker", 1);
    ros::Publisher RF_pub = nh.advertise<visualization_msgs::Marker>("/RF_marker", 1);

    ros::Rate rate(100);
    while (ros::ok()) {
        body_pub.publish(com_marker_msg);
        LH_pub.publish(lh_marker_msg);
        LF_pub.publish(lf_marker_msg);
        RH_pub.publish(rh_marker_msg);
        RF_pub.publish(rf_marker_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
