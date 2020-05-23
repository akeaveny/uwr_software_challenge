//ros
#include "ros/ros.h"    
#include "turtlesim/Pose.h"	
#include "geometry_msgs/Twist.h"

#include "software_challenge/TwoTurtlesDist.h"

// std lib
#include <cstdlib>

// fwd declarations
ros::Publisher pub;
double getDistance(double x1, double y1, double x2, double y2);
void StationaryPoseCallback(const turtlesim::Pose::ConstPtr& msg);
void MovingPoseCallback(const turtlesim::Pose::ConstPtr& msg);													
turtlesim::Pose StationaryPose;												
turtlesim::Pose MovingPose;

// distance between two points
double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

// call backs
void StationaryPoseCallback(const turtlesim::Pose::ConstPtr& msg)			
{
	StationaryPose.x = msg->x;
	StationaryPose.y = msg->y;
    // ROS_INFO("x: %f  y: %f", StationaryPose.x, StationaryPose.y);										
	return;
}

void MovingPoseCallback(const turtlesim::Pose::ConstPtr& msg)			
{
	MovingPose.x = msg->x;
	MovingPose.y = msg->y;
    // ROS_INFO("x: %f  y: %f", MovingPose.x, MovingPose.y);										
	return;
}


int main(int argc, char **argv) {

    //init
    ros::init(argc, argv, "publish_pose");
    ros::NodeHandle nh;

    ros::Subscriber CurPose1_sub = nh.subscribe("/stationary_turtle/pose", 10, StationaryPoseCallback);
    ros::Subscriber MovingPose_sub = nh.subscribe("/moving_turtle/pose", 10, MovingPoseCallback);

    ros::Publisher pose_publisher = nh.advertise<software_challenge::TwoTurtlesDist>("two_turtles_pose", 1);

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        
        software_challenge::TwoTurtlesDist msg;
        msg.distance_x = MovingPose.x - StationaryPose.x;
        msg.distance_y = MovingPose.y - StationaryPose.y;
        msg.distance_rel = getDistance(StationaryPose.x, StationaryPose.y, MovingPose.x, MovingPose.y);

        ROS_INFO("X: [%f]", msg.distance_x);
        ROS_INFO("Y: [%f]", msg.distance_y);
        ROS_INFO("Relative: [%f]", msg.distance_rel);

        pose_publisher.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}