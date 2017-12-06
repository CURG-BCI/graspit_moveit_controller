cmake_minimum_required(VERSION 2.8.3)
project(graspit_moveit_controller)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation
                                        trajectory_msgs moveit_msgs geometry_msgs graspit_msgs actionlib)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS actionlib
)
