// ros.h
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>

// std lib
#include <cstdlib>

void killturtle(const std::string &name = "turtle1") {

    ros::NodeHandle nh_;
    ros::ServiceClient killturtle = nh_.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill srv;
    // ROS_INFO_STREAM(name);
    srv.request.name = name;
    killturtle.call(srv);
}

void spawnturtle(float x, float y, float theta, const std::string &name) {

    ros::NodeHandle nh_;
    ros::ServiceClient spawnturtle = nh_.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.theta = theta;
    srv.request.name = name;
    spawnturtle.call(srv);
}

int main(int argc, char **argv) {

    //init
    ros::init(argc, argv, "spawn_turtles");
    ros::NodeHandle nh;


    // ====== clear ======   
    killturtle();

    // ====== spawn ====== 
    spawnturtle(5, 5, 0, "stationary_turtle");
    spawnturtle(25, 10, 0, "moving_turtle");
}