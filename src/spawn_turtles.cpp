// ros.h
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>

// std lib
#include <cstdlib>
#include <string>

// std::string turtle1_name_;
// double turtle1_x_;
// double turtle1_y_;

void killturtle(const std::string &name = "turtle1") {

    ros::NodeHandle nh_;
    ros::ServiceClient killturtle = nh_.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill srv;
    // ROS_INFO_STREAM(name);
    srv.request.name = name;
    killturtle.call(srv);
}

void spawnturtle(float x, float y, float theta, const std::string &name) {

    ROS_INFO_STREAM("Name: " << name);
    ROS_INFO_STREAM("X: " << x);
    ROS_INFO_STREAM("Y: " << y);

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

    XmlRpc::XmlRpcValue turtles_;
    nh.getParam("/uwr_software_challenge/spawn_turtles/turtles", turtles_);

    for (int turtle_index = 0; turtle_index < turtles_.size(); turtle_index++) {
        ROS_ASSERT(turtles_[turtle_index].getType() == XmlRpc::XmlRpcValue::TypeStruct);

        ROS_ASSERT(turtles_[turtle_index].hasMember("name"));
        ROS_ASSERT(turtles_[turtle_index]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string turtle_name_ = turtles_[turtle_index]["name"];

        ROS_ASSERT(turtles_[turtle_index].hasMember("x"));
        ROS_ASSERT(turtles_[turtle_index]["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double turtle_x_ = turtles_[turtle_index]["x"];

        ROS_ASSERT(turtles_[turtle_index].hasMember("y"));
        ROS_ASSERT(turtles_[turtle_index]["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double turtle_y_ = turtles_[turtle_index]["y"];

        // ====== spawn ====== 
        spawnturtle(turtle_x_, turtle_y_, 0, turtle_name_);
        
    }

    // nh.getParam("/uwr_software_challenge/spawn_turtles/turtle1_x_", turtle1_x_);
    // nh.getParam("/uwr_software_challenge/spawn_turtles/turtle1_y_", turtle1_y_);

    // ====== spawn ====== 
    // spawnturtle(turtle1_x_, turtle1_y_, 0, turtle1_name_);
    // spawnturtle(25, 10, 0, "moving_turtle");
}