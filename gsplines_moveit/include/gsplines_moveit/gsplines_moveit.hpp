#ifndef GSPLINES_MOVEIT
#define GSPLINES_MOVEIT
#include <eigen3/Eigen/Core>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
Eigen::MatrixXd
robot_trajectory_waypoints(const moveit_msgs::RobotTrajectory &_msg);

Eigen::MatrixXd
robot_trajectory_waypoints(const robot_trajectory::RobotTrajectory &_msg);

Eigen::MatrixXd motion_plan_response_waypoints(
    const planning_interface::MotionPlanResponse &_plan);

#endif
