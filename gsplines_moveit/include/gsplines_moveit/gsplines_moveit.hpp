#ifndef GSPLINES_MOVEIT
#define GSPLINES_MOVEIT
#include <eigen3/Eigen/Core>
#include <gsplines/Basis/Basis.hpp>
#include <gsplines/GSpline.hpp>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <optional>
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

bool compute_minimum_sobolev_semi_norm_robot_trajectory(
    robot_trajectory::RobotTrajectory &_trj,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights, double _exec_time,
    const ros::Duration &_step,
    const std_msgs::Header _header = std_msgs::Header());

/// Joint waypoints in \ref _trj with a sobolev-semi-norm-optimal trajectory
/// linearly scaled to minimize the time
bool compute_minimum_sobolev_semi_norm_robot_trajectory(
    robot_trajectory::RobotTrajectory
        &_trj, //< trajectory containing the waypoints
    const gsplines::basis::Basis &_basis, //< basis
    std::vector<std::pair<std::size_t, double>>
        _weights,               //< optimization weights
    const ros::Duration &_step, //< time step between waypoints
    double _vel_factor = 1.0,   //< velocity scaling
    double _acc_factor = 1.0,   // < acceleration scaling
    const std::optional<double> &_exec_time = std::nullopt //
);

bool compute_minimum_jerk_trajectory(robot_trajectory::RobotTrajectory &_trj,
                                     const ros::Duration &_step,
                                     double _vel_factor = 1.0,
                                     double _acc_factor = 1.0);

gsplines::GSpline
interpolate_robot_trajectory(const moveit_msgs::RobotTrajectory &_msg,
                             const gsplines::basis::Basis &_basis);
} // namespace gsplines_moveit
#endif
