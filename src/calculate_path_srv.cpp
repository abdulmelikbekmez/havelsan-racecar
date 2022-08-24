#include "ros/ros.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>

#define CONSTANT 1.15

ackermann_msgs::AckermannDriveStamped msg_drive;
ros::Publisher pub;

void callback(const geometry_msgs::TwistConstPtr &msg)
{
    msg_drive.drive.speed = msg->linear.x;
    msg_drive.drive.steering_angle = msg->angular.z * CONSTANT;
    pub.publish(msg_drive);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ackerman_drive_publisher");
    ros::NodeHandle n;
    pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd_mux/input/navigation", 1);
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1, callback);
    ros::spin();
    return 0;
}
