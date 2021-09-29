#ifndef GSPLINES_MOVEIT
#define GSPLINES_MOVEIT
#include <eigen3/Eigen/Core>
#include <gsplines/Basis/Basis.hpp>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
namespace gsplines_moveit {

Eigen::MatrixXd
robot_trajectory_waypoints(const moveit_msgs::RobotTrajectory &_msg);

Eigen::MatrixXd
robot_trajectory_waypoints(const robot_trajectory::RobotTrajectory &_msg);

Eigen::MatrixXd motion_plan_response_waypoints(
    const planning_interface::MotionPlanResponse &_plan);

robot_trajectory::RobotTrajectory minimum_sobolev_semi_norm_robot_trajectory(
    const robot_trajectory::RobotTrajectory &_trj,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights, double _exec_time,
    const ros::Duration &_step,
    const std_msgs::Header _header = std_msgs::Header());
} // namespace gsplines_moveit
#endif
