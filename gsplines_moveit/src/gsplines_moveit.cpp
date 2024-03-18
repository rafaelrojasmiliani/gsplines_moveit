#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Optimization/ipopt_solver.hpp>
#include <gsplines_moveit/gsplines_moveit.hpp>
#include <gsplines_ros/gsplines_ros.hpp>

namespace gsplines_moveit {

struct boundVector {
  std::vector<double> acceleration_bounds;
  std::vector<double> velocity_bounds;
};

boundVector getBounds(const moveit::core::JointModelGroup &group,
                      double _vel_factor = 1.0, double _acc_factor = 1.0) {
  const auto &joint_names = group.getVariableNames();
  const auto &rmodel = group.getParentModel();
  std::vector<double> velocity_bounds;
  std::vector<double> acceleration_bounds;
  for (const auto &join_name : joint_names) {
    const auto &bounds = rmodel.getVariableBounds(join_name);

    // set velocity bounds
    if (bounds.velocity_bounded_) {
      const double max_v = std::max(std::abs(bounds.max_velocity_),
                                    std::abs(bounds.min_velocity_));
      velocity_bounds.push_back(max_v);
    } else {
      velocity_bounds.push_back(_vel_factor);
    }

    // set acceleration bounds
    if (bounds.acceleration_bounded_) {
      const double max_v = std::max(std::abs(bounds.max_acceleration_),
                                    std::abs(bounds.min_acceleration_));
      acceleration_bounds.push_back(max_v);
    } else {
      acceleration_bounds.push_back(_acc_factor);
    }
  }
  return {std::move(velocity_bounds), std::move(acceleration_bounds)};
}

static const std::string LOGNAME = "gsplines_moveit";

robot_trajectory::RobotTrajectory function_to_robot_trajectory(
    const gsplines::functions::FunctionBase &_trj,
    const robot_model::RobotModelConstPtr &robot_model,
    const moveit::core::RobotState &reference_state, const std::string &group,
    const std::vector<std::string> &_joint_names, const ros::Duration &_rate) {

  robot_trajectory::RobotTrajectory result(robot_model, group);

  trajectory_msgs::JointTrajectory joint_trajectory =
      gsplines_ros::function_to_joint_trajectory_msg(_trj, _joint_names, _rate);

  result.setRobotTrajectoryMsg(reference_state, joint_trajectory);

  return result;
}

Eigen::MatrixXd robot_trajectory_waypoints(moveit_msgs::RobotTrajectory &_msg) {
  return gsplines_ros::get_waypoints(_msg.joint_trajectory);
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
    const robot_trajectory::RobotTrajectory &_trj,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights, double _exec_time,
    const ros::Duration &_step, const std_msgs::Header _header) {

  const moveit::core::JointModelGroup *group = _trj.getGroup();
  const moveit::core::RobotModel &rmodel = group->getParentModel();

  // const std::vector<int> &idx = group->getVariableIndexList();
  const std::vector<std::string> &joint_names = group->getVariableNames();

  std::vector<double> velocity_bounds;
  std::vector<double> acceleration_bounds;
  std::transform(joint_names.begin(), joint_names.end(),
                 std::back_inserter(velocity_bounds),
                 [rmodel](const std::string &_var_name) {
                   return rmodel.getVariableBounds(_var_name).max_velocity_;
                 });

  std::transform(joint_names.begin(), joint_names.end(),
                 std::back_inserter(acceleration_bounds),
                 [rmodel](const std::string &_var_name) {
                   return rmodel.getVariableBounds(_var_name).max_acceleration_;
                 });

  Eigen::MatrixXd waypoints = robot_trajectory_waypoints(_trj);

  trajectory_msgs::JointTrajectory joint_trajectory =
      gsplines_ros::minimum_sobolev_semi_norm_joint_trajectory(
          waypoints, joint_names, _basis, _weights, _exec_time, _step, _header);

  robot_trajectory::RobotTrajectory result(_trj, true);

  result.setRobotTrajectoryMsg(_trj.getFirstWayPoint(), joint_trajectory);

  return result;
}

bool compute_minimum_sobolev_semi_norm_robot_trajectory(
    robot_trajectory::RobotTrajectory &_trj,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights, double _exec_time,
    const ros::Duration &_step, const std_msgs::Header _header) {

  // 1. Get joint names
  const std::vector<std::string> &joint_names =
      _trj.getGroup()->getParentModel().getVariableNames();

  // 2. Get waypoints

  Eigen::MatrixXd waypoints = robot_trajectory_waypoints(_trj);

  // 3. optimize
  trajectory_msgs::JointTrajectory joint_trajectory =
      gsplines_ros::minimum_sobolev_semi_norm_joint_trajectory(
          waypoints, joint_names, _basis, _weights, _exec_time, _step, _header);

  // 5. Set pitüit from trajectory message
  // 5.1 copy the first state to have a reference.
  const moveit::core::RobotState copy(_trj.getFirstWayPoint());
  _trj.setRobotTrajectoryMsg(copy, joint_trajectory);
  return true;
}
bool compute_minimum_sobolev_semi_norm_robot_trajectory(
    robot_trajectory::RobotTrajectory &_trj,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights,
    const ros::Duration &_step, //< time step between waypoints
    double _vel_factor,         //< velocity scaling
    double _acc_factor,         // < acceleration scaling
    const std::optional<double> &_exec_time) {
  // 1. Get the robot model from the moveit's robot_trajectory
  const moveit::core::JointModelGroup *group = _trj.getGroup();

  // const std::vector<int> &idx = group->getVariableIndexList();
  const std::vector<std::string> &joint_names = group->getVariableNames();

  // 2. Get upper velocity and acceleration bounds of the trajectory
  auto bounds = getBounds(*group, _vel_factor, _acc_factor);
  // 3. Get waypoints from trajecotry
  Eigen::MatrixXd waypoints = robot_trajectory_waypoints(_trj);

  // 4. optimize, get message
  trajectory_msgs::JointTrajectory joint_trajectory =
      gsplines_ros::minimum_sobolev_semi_norm_joint_trajectory(
          waypoints, joint_names, _basis, _weights, bounds.velocity_bounds,
          bounds.acceleration_bounds, _step, _exec_time);

  // 5. Set pitüit from trajectory message
  // 5.1 copy the first state to have a reference.
  const moveit::core::RobotState copy(_trj.getFirstWayPoint());
  _trj.setRobotTrajectoryMsg(copy, joint_trajectory);

  return true;
}

bool compute_minimum_jerk_trajectory(robot_trajectory::RobotTrajectory &_trj,
                                     const ros::Duration &_step,
                                     double _vel_factor, double _acc_factor) {

  // 1. Get the robot model from the moveit's robot_trajectory
  const moveit::core::JointModelGroup *group = _trj.getGroup();

  // const std::vector<int> &idx = group->getVariableIndexList();
  const std::vector<std::string> &joint_names = group->getVariableNames();

  // 2. Get upper velocity and acceleration bounds of the trajectory
  auto bounds = getBounds(*group, _vel_factor, _acc_factor);
  // 3. Get waypoints from trajecotry
  Eigen::MatrixXd waypoints = robot_trajectory_waypoints(_trj);

  // 4. optimize, get message
  trajectory_msgs::JointTrajectory joint_trajectory =
      gsplines_ros::minimum_jerk_trajectory(waypoints, joint_names,
                                            bounds.velocity_bounds,
                                            bounds.acceleration_bounds, _step);

  // 5. Set pitüit from trajectory message
  // 5.1 copy the first state to have a reference.
  const moveit::core::RobotState copy(_trj.getFirstWayPoint());
  _trj.setRobotTrajectoryMsg(copy, joint_trajectory);

  return true;
}
} // namespace gsplines_moveit
