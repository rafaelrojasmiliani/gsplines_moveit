#ifndef GSPLINES_MOVEIT
#define GSPLINES_MOVEIT
#include <eigen3/Eigen/Core>
#include <gsplines/Basis/Basis.hpp>
#include <gsplines/GSpline.hpp>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <optional>
namespace gsplines_moveit {

Eigen::MatrixXd
robot_trajectory_waypoints(const moveit_msgs::RobotTrajectory &_msg);

Eigen::MatrixXd filterCollinearWaypoints(const Eigen::MatrixXd &_mat);

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

std::vector<gsplines::GSpline>
forward_kinematics_frames(const gsplines::GSpline &joint_trj,
                          const moveit::core::JointModelGroup *group,
                          const moveit::core::RobotModelPtr &_model,
                          const std::vector<std::string> &_links);

double get_max_frame_speed(const gsplines::GSpline &joint_trj,
                           const moveit::core::JointModelGroup *group,
                           const moveit::core::RobotModelPtr &_model,
                           const std::vector<std::string> &_links,
                           std::size_t nglp = 13, std::size_t nintervals = 10);

double get_max_frame_speed(const gsplines::GSpline &joint_trj,
                           const moveit::core::JointModelGroup *group,
                           const moveit::core::RobotModelConstPtr &_model,
                           double step = 0.001);

gsplines::GSpline
scale_trajectory(const gsplines::GSpline &trj,
                 const planning_interface::MotionPlanRequest &req,
                 planning_interface::MotionPlanResponse &res);
} // namespace gsplines_moveit
#endif
