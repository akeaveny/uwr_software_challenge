#include "ros/ros.h"
#include "software_training_assignment/ResetTurtle.h"
#include "turtlesim/TeleportAbsolute.h"
#include <cmath>										
#include "turtlesim/Pose.h"	
#include "geometry_msgs/Twist.h"
#include <time.h>
#include <software_training_assignment/TurtleAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<software_training_assignment::TurtleAction> Server;

//fwd declarations
ros::ServiceClient client;
ros::Publisher pub;
double getDistance(double x1, double y1, double x2, double y2);
void CurPose1Callback(const turtlesim::Pose::ConstPtr& msg);
void CurPose2Callback(const turtlesim::Pose::ConstPtr& msg);
bool reset(software_training_assignment::ResetTurtle::Request &req,
         software_training_assignment::ResetTurtle::Response &res);
void execute(const software_training_assignment::TurtleGoalConstPtr& goal, Server* as);


// Global variables
//bool STOP = true;														
turtlesim::Pose CurPose1;												
turtlesim::Pose CurPose2;


//main
int main(int argc, char **argv)
{
  //init
  ros::init(argc, argv, "turtle_node");
  ros::NodeHandle n;

  //reset turtle server
  client = n.serviceClient<turtlesim::TeleportAbsolute>("moving_turtle/teleport_absolute");
  ros::ServiceServer service = n.advertiseService("turtle_reset", reset);

  //ROS_INFO("Ready to Reset Turtle.");

  //publisher for custom msg
  ros::Subscriber CurPose1_sub = n.subscribe("/stationary_turtle/pose", 5, CurPose1Callback); 	//stationary_turtle/pose
  ros::Subscriber CurPose2_sub = n.subscribe("/moving_turtle/pose", 5, CurPose2Callback); 	//moving_turtle/pose

  ROS_INFO("Please enter waypoint goal");
  	
  //action server
  Server server(n, "turtle_node", boost::bind(&execute, _1, &server), false);
  server.start();

  // A publisher for the movement data
  pub = n.advertise<geometry_msgs::Twist>("moving_turtle/cmd_vel", 10);

  //loop rate
  //ros::Rate rate(1);
  while (ros::ok()) 
{
  //rate.sleep();
  ros::spin();
  return 0;
}

  ros::spin();

  return 0;
}


//service to reset pose
bool reset(software_training_assignment::ResetTurtle::Request &req,
         software_training_assignment::ResetTurtle::Response &res)
{

  //input to reset turtle service class
  turtlesim::TeleportAbsolute srv;
  srv.request.x = req.x;
  srv.request.y = req.y;
  srv.request.theta = req.theta;
  res.call = true; 

if (client.call(srv))
  {
  }
  
  else
  {
    res.call = false;
  }

  return true;
}


//distance between two points
double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}


// call back to send new current Pose msgs
void CurPose1Callback(const turtlesim::Pose::ConstPtr& msg)			
{
	CurPose1.x = msg->x;
	CurPose1.y = msg->y;										
	return;
}

// call back to send new current Pose msgs
void CurPose2Callback(const turtlesim::Pose::ConstPtr& msg)			
{
	CurPose2.x = msg->x;
	CurPose2.y = msg->y;										
	return;
}

//action
void execute(const software_training_assignment::TurtleGoalConstPtr& goal, Server* as)
{
	 //parameters
	 const double speed = -0.5;
	 double t0 = ros::Time::now().toSec();
	 double t1 = ros::Time::now().toSec();
	 double distance;
         double current_distance = 0;

         // Drive forward at a given speed
	 geometry_msgs::Twist msg;
	 msg.linear.x = speed;

	//action file
 	distance = goal->waypoint;
	software_training_assignment::TurtleResult result;
	software_training_assignment::TurtleFeedback feedback;

         //Loop at 1Hz to move the turtle in an specified distance
         ros::Rate loop_rate(1);

         while(abs(current_distance) < distance)
{	         
            //Publish the velocity
            pub.publish(msg);
            //Takes actual time to velocity calculus
            t1 = ros::Time::now().toSec();
            //Calculates distancePoseStamped
            current_distance= speed*(t1-t0);
	    //printing current_distance
	    //ROS_INFO("t0 = %lf, t1 = %lf, distance = %lf, current_distance = %lf" ,t0, t1, distance, current_distance);
	    feedback.distance = getDistance(distance - abs(current_distance), 0, 0, 0);
            ROS_INFO("Distance to the goal = %lf", feedback.distance);

	    //print x, y and rel distances
	    ROS_INFO("x = %.2f, y= %.2f, rel=%.2f\n", CurPose1.x - CurPose2.x, CurPose1.y - CurPose2.y,  getDistance(CurPose1.x, CurPose2.x, CurPose1.y, CurPose2.y));
	
	    loop_rate.sleep();
}
	    
	    //After the loop, stops the robot
            msg.linear.x = 0;

            //Force the robot to stop
            pub.publish(msg);

	   //result 
	   result.time_elapsed = ros::Duration(t1-t0);
	   ROS_INFO("Time it took to reach the goal = %lf", t1-t0);

          //set action to succeed
  	  as->setSucceeded(result);
}

