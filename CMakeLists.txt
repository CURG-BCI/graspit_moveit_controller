cmake_minimum_required(VERSION 2.8.3)
project(moveit_trajectory_planner)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation
                                        trajectory_msgs moveit_msgs geometry_msgs graspit_msgs actionlib model_rec2)


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   LocationInfo.srv
   BoxInfo.srv
   MeshInfo.srv
   ObjectName.srv

 )

 ## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   trajectory_msgs
   moveit_msgs
   geometry_msgs
   graspit_msgs
   actionlib_msgs
 )

catkin_package(
  CATKIN_DEPENDS actionlib
)
