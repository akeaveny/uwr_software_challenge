// ros.h
#include "ros/ros.h"
#include <time.h>

#include "turtlesim/Pose.h"	
#include "geometry_msgs/Twist.h"

#include <uwr_software_challenge/MoveTurtleAction.h>
#include <actionlib/server/simple_action_server.h>

// std lib
#include <cstdlib>

typedef actionlib::SimpleActionServer<uwr_software_challenge::MoveTurtleAction> Server;
uwr_software_challenge::MoveTurtleResult result;
uwr_software_challenge::MoveTurtleFeedback feedback;

ros::Subscriber pose_sub;
ros::Publisher cmd_vel_pub;
void action_callback(const uwr_software_challenge::MoveTurtleGoalConstPtr &goal, Server* as);

float moving_turtle_x, moving_turtle_y;
void setMovingTurtlePose(const turtlesim::Pose::ConstPtr& msg) {
	moving_turtle_x = msg->x;
	moving_turtle_y = msg->y;
}

//distance between two points
double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}


int main(int argc, char **argv) {

    //init
    ros::init(argc, argv, "move_turtles");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/moving_turtle/pose", 1000, setMovingTurtlePose);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("moving_turtle/cmd_vel", 1000);

    //action server
    Server as(nh, "move_turtles", boost::bind(&action_callback, _1, &as), false);
    as.start();

    ros::spin();

    return 0;
}

void action_callback(const uwr_software_challenge::MoveTurtleGoalConstPtr &goal, Server* as) {
	 
    const double linear_speed = 0.5;
    const double angular_speed = 0.5;
    double t0 = ros::Time::now().toSec();
    double t1 = ros::Time::now().toSec();
    double distance;
    double current_distance = 0;

    // turn in place
	geometry_msgs::Twist msg;
    msg.linear.x = msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y = 0;
    msg.angular.z = angular_speed;

    ros::Rate loop_rate(100);

    ros::Time begin = ros::Time::now();
    loop_rate.sleep();

    cmd_vel_pub.publish(msg);

	// axclient
    distance = getDistance(goal->goal_x, goal->goal_y, 0, 0);

    while(abs(current_distance) < distance) {	         

        t1 = ros::Time::now().toSec();
        current_distance= linear_speed*(t1-t0);

         // drive forward
        geometry_msgs::Twist msg;
        msg.angular.z = msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y = 0;
        msg.linear.x = linear_speed;
        cmd_vel_pub.publish(msg);

        feedback.distance_to_goal = getDistance(distance - abs(current_distance), 0, 0, 0);
        ROS_INFO("Distance to the goal = %lf", feedback.distance_to_goal);

        loop_rate.sleep();
    }

    // stop the robot
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub.publish(msg);

    // print time 
    result.time_taken = (ros::Time::now() - begin).toSec();
    ROS_INFO("Time it took to reach the goal = %lf", t1-t0);

    //set action to succeed
    as->setSucceeded(result);

}
    