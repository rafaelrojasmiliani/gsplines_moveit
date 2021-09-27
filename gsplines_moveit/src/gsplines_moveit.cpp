#include <gsplines_moveit/gsplines_moveit.hpp>
#include <gsplines_ros/gsplines_ros.hpp>

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
