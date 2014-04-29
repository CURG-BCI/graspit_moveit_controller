#!/bin/bash
rosnode kill /grasp_analyzer
rosrun moveit_trajectory_planner grasp_analyzer_node.py