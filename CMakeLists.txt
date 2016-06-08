cmake_minimum_required(VERSION 2.8.3)
project(moveit_trajectory_planner)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation
                                        trajectory_msgs moveit_msgs geometry_msgs graspit_msgs model_rec2 actionlib)


catkin_package(
  CATKIN_DEPENDS actionlib
)
