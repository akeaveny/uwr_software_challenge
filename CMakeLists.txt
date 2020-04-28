cmake_minimum_required(VERSION 2.8.3)
project(software_training_assignment)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp actionlib turtlesim std_msgs message_generation actionlib_msgs)

## Declare ROS messages and services
add_service_files(FILES ResetTurtle.srv)

## Declare ROS action files
add_action_files(DIRECTORY action FILES Turtle.action)

## Generate added messages and services
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

## Declare a catkin package
## catkin_package(CATKIN_DEPENDS actionlib_msgs message_runtime)
catkin_package(CATKIN_DEPENDS actionlib_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(turtle_node src/turtle_node)
target_link_libraries(turtle_node ${catkin_LIBRARIES})
add_dependencies(turtle_node software_training_assignment_generate_messages_cpp)
