#include <gsplines/Optimization/ipopt_solver.hpp>
#include <gsplines_moveit/gsplines_moveit.hpp>
#include <gsplines_ros/gsplines_ros.hpp>

namespace gsplines_moveit {
Eigen::MatrixXd robot_trajectory_waypoints(moveit_msgs::RobotTrajectory &_msg) {
  return gsplines_ros::waypoint_matrix(_msg.joint_trajectory);
}

Eigen::MatrixXd
motion_plan_response_waypoints(planning_interface::MotionPlanResponse &_plan) {

  return robot_trajectory_waypoints(*_plan.trajectory_);
}

Eigen::MatrixXd
robot_trajectory_waypoints(const robot_trajectory::RobotTrajectory &_msg) {

  const robot_model::JointModelGroup *jmg = _msg.getGroup();

  std::size_t codom_dim = jmg->getVariableNames().size();

  std::size_t nw = _msg.getWayPointCount();

  Eigen::MatrixXd result(nw, codom_dim);

  std::vector<double> joint_values;
  for (std::size_t i = 0; i < nw; i++) {
    _msg.getWayPoint(i).copyJointGroupPositions(jmg, joint_values);

    result.row(i) = Eigen::Map<const Eigen::Matrix<double, 1, Eigen::Dynamic>>(
        joint_values.data(), codom_dim);
  }

  return result;
}

gsplines::GSpline
minimum_sobolev_semi_norm(const moveit_msgs::RobotTrajectory &_msg,
                          const gsplines::basis::Basis &_basis,
                          std::vector<std::pair<std::size_t, double>> _weights,
                          double _exec_time) {

  Eigen::MatrixXd waypoint = robot_trajectory_waypoints(_msg);

  return gsplines::optimization::optimal_sobolev_norm(waypoint, _basis,
                                                      _weights, _exec_time);
}

gsplines::GSpline
minimum_sobolev_semi_norm(const robot_trajectory::RobotTrajectory &_trj,
                          const gsplines::basis::Basis &_basis,
                          std::vector<std::pair<std::size_t, double>> _weights,
                          double _exec_time) {

  Eigen::MatrixXd waypoint = robot_trajectory_waypoints(_trj);

  return gsplines::optimization::optimal_sobolev_norm(waypoint, _basis,
                                                      _weights, _exec_time);
}

robot_trajectory::RobotTrajectory minimum_sobolev_semi_norm_robot_trajectory(
    const moveit_msgs::RobotTrajectory &_msg,
    const robot_model::RobotModelConstPtr &robot_model,
    const std::string &group, const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights, double _exec_time) {

  robot_trajectory::RobotTrajectory result(robot_model, group);
}

robot_trajectory::RobotTrajectory minimum_sobolev_semi_norm_robot_trajectory(
    const robot_trajectory::RobotTrajectory &_trj,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights, double _exec_time) {}

robot_trajectory::RobotTrajectory minimum_sobolev_semi_norm_robot_trajectory(
    const Eigen::MatrixXd &, const robot_model::RobotModelConstPtr &robot_model,
    const std::string &group, const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights, double _exec_time) {}

robot_trajectory::RobotTrajectory minimum_sobolev_semi_norm_robot_trajectory(
    const Eigen::MatrixXd &, const robot_model::RobotModelConstPtr &robot_model,
    const robot_model::JointModelGroup *group,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights, double _exec_time) {}

} // namespace gsplines_moveit
