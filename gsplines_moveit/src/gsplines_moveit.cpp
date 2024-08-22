#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Collocation/GaussLobattoLagrange.hpp>
#include <gsplines/Optimization/ipopt_solver.hpp>
#include <gsplines_moveit/gsplines_moveit.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <moveit/planning_interface/planning_request.h>

namespace gsplines_moveit {

struct boundVector {
  std::vector<double> acceleration_bounds;
  std::vector<double> velocity_bounds;
};

Eigen::MatrixXd filterCollinearWaypoints(const Eigen::MatrixXd &_mat) {

  Eigen::IOFormat OctaveFmt(6, 0, ", ", ",\n", "[", "]", "[", "]");

  std::vector<long> points;

  points.emplace_back(0);

  Eigen::RowVectorXd direction = (_mat.row(1) - _mat.row(0)).normalized();
  Eigen::RowVectorXd d(direction);

  for (int i = 2; i < _mat.rows(); ++i) {
    d = (_mat.row(i) - _mat.row(points.back())).normalized();

    if (!d.isApprox(direction)) {
      points.emplace_back(i - 1);
      direction = d;
    }
  }

  points.emplace_back(_mat.rows() - 1);

  Eigen::MatrixXd result(points.size(), _mat.cols());
  result.setZero();

  for (int i = 0; i < static_cast<int>(points.size()); i++) {
    ROS_WARN_STREAM("------------------\n" << result.format(OctaveFmt));
    result.row(i) = _mat.row(points[i]);
    ROS_WARN_STREAM("------------------\n"
                    << result.format(OctaveFmt)
                    << "\n------------------\n.------------------");
  }
  return result;
}

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

    result.row(static_cast<long>(i)) =
        Eigen::Map<const Eigen::Matrix<double, 1, Eigen::Dynamic>>(
            joint_values.data(), static_cast<long>(codom_dim));
  }

  return result;
}

// gsplines::GSpline
// minimum_sobolev_semi_norm(const moveit_msgs::RobotTrajectory &_msg,
//                           const gsplines::basis::Basis &_basis,
//                           std::vector<std::pair<std::size_t, double>>
//                           _weights, double _exec_time) {

//   Eigen::MatrixXd waypoint = robot_trajectory_waypoints(_msg);

//   return gsplines::optimization::optimal_sobolev_norm(waypoint, _basis,
//                                                       _weights, _exec_time);
// }

// gsplines::GSpline
// minimum_sobolev_semi_norm(const robot_trajectory::RobotTrajectory &_trj,
//                           const gsplines::basis::Basis &_basis,
//                           std::vector<std::pair<std::size_t, double>>
//                           _weights, double _exec_time) {

//   Eigen::MatrixXd waypoint = robot_trajectory_waypoints(_trj);

//   return gsplines::optimization::optimal_sobolev_norm(waypoint, _basis,
//                                                       _weights, _exec_time);
// }

// robot_trajectory::RobotTrajectory minimum_sobolev_semi_norm_robot_trajectory(
//     const robot_trajectory::RobotTrajectory &_trj,
//     const gsplines::basis::Basis &_basis,
//     std::vector<std::pair<std::size_t, double>> _weights, double _exec_time,
//     const ros::Duration &_step, const std_msgs::Header _header) {

//   const moveit::core::JointModelGroup *group = _trj.getGroup();
//   const moveit::core::RobotModel &rmodel = group->getParentModel();

//   // const std::vector<int> &idx = group->getVariableIndexList();
//   const std::vector<std::string> &joint_names = group->getVariableNames();

//   std::vector<double> velocity_bounds;
//   std::vector<double> acceleration_bounds;
//   std::transform(joint_names.begin(), joint_names.end(),
//                  std::back_inserter(velocity_bounds),
//                  [rmodel](const std::string &_var_name) {
//                    return rmodel.getVariableBounds(_var_name).max_velocity_;
//                  });

//   std::transform(joint_names.begin(), joint_names.end(),
//                  std::back_inserter(acceleration_bounds),
//                  [rmodel](const std::string &_var_name) {
//                    return
//                    rmodel.getVariableBounds(_var_name).max_acceleration_;
//                  });

//   Eigen::MatrixXd waypoints = robot_trajectory_waypoints(_trj);

//   trajectory_msgs::JointTrajectory joint_trajectory =
//       gsplines_ros::minimum_sobolev_semi_norm_joint_trajectory(
//           waypoints, joint_names, _basis, _weights, _exec_time, _step,
//           _header);

//   robot_trajectory::RobotTrajectory result(_trj, true);

//   result.setRobotTrajectoryMsg(_trj.getFirstWayPoint(), joint_trajectory);

//   return result;
// }

// bool compute_minimum_sobolev_semi_norm_robot_trajectory(
//     robot_trajectory::RobotTrajectory &_trj,
//     const gsplines::basis::Basis &_basis,
//     std::vector<std::pair<std::size_t, double>> _weights, double _exec_time,
//     const ros::Duration &_step, const std_msgs::Header _header) {

//   // 1. Get joint names
//   const std::vector<std::string> &joint_names =
//       _trj.getGroup()->getParentModel().getVariableNames();

//   // 2. Get waypoints

//   Eigen::MatrixXd waypoints = robot_trajectory_waypoints(_trj);

//   // 3. optimize
//   trajectory_msgs::JointTrajectory joint_trajectory =
//       gsplines_ros::minimum_sobolev_semi_norm_joint_trajectory(
//           waypoints, joint_names, _basis, _weights, _exec_time, _step,
//           _header);

//   // 5. Set pitüit from trajectory message
//   // 5.1 copy the first state to have a reference.
//   const moveit::core::RobotState copy(_trj.getFirstWayPoint());
//   _trj.setRobotTrajectoryMsg(copy, joint_trajectory);
//   return true;
// }
// bool compute_minimum_sobolev_semi_norm_robot_trajectory(
//     robot_trajectory::RobotTrajectory &_trj,
//     const gsplines::basis::Basis &_basis,
//     std::vector<std::pair<std::size_t, double>> _weights,
//     const ros::Duration &_step, //< time step between waypoints
//     double _vel_factor,         //< velocity scaling
//     double _acc_factor,         // < acceleration scaling
//     const std::optional<double> &_exec_time) {
//   // 1. Get the robot model from the moveit's robot_trajectory
//   const moveit::core::JointModelGroup *group = _trj.getGroup();

//   // const std::vector<int> &idx = group->getVariableIndexList();
//   const std::vector<std::string> &joint_names = group->getVariableNames();

//   // 2. Get upper velocity and acceleration bounds of the trajectory
//   auto bounds = getBounds(*group, _vel_factor, _acc_factor);
//   // 3. Get waypoints from trajecotry
//   Eigen::MatrixXd waypoints = robot_trajectory_waypoints(_trj);

//   // 4. optimize, get message
//   trajectory_msgs::JointTrajectory joint_trajectory =
//       gsplines_ros::minimum_sobolev_semi_norm_joint_trajectory(
//           waypoints, joint_names, _basis, _weights, bounds.velocity_bounds,
//           bounds.acceleration_bounds, _step, _exec_time);

//   // 5. Set pitüit from trajectory message
//   // 5.1 copy the first state to have a reference.
//   const moveit::core::RobotState copy(_trj.getFirstWayPoint());
//   _trj.setRobotTrajectoryMsg(copy, joint_trajectory);

//   return true;
// }

// bool compute_minimum_jerk_trajectory(robot_trajectory::RobotTrajectory &_trj,
//                                      const ros::Duration &_step,
//                                      double _vel_factor, double _acc_factor)
//                                      {

//   // 1. Get the robot model from the moveit's robot_trajectory
//   const moveit::core::JointModelGroup *group = _trj.getGroup();

//   // const std::vector<int> &idx = group->getVariableIndexList();
//   const std::vector<std::string> &joint_names = group->getVariableNames();

//   // 2. Get upper velocity and acceleration bounds of the trajectory
//   auto bounds = getBounds(*group, _vel_factor, _acc_factor);
//   // 3. Get waypoints from trajecotry
//   Eigen::MatrixXd waypoints = robot_trajectory_waypoints(_trj);

//   // 4. optimize, get message
//   trajectory_msgs::JointTrajectory joint_trajectory =
//       gsplines_ros::minimum_jerk_trajectory(waypoints, joint_names,
//                                             bounds.velocity_bounds,
//                                             bounds.acceleration_bounds,
//                                             _step);

//   // 5. Set pitüit from trajectory message
//   // 5.1 copy the first state to have a reference.
//   const moveit::core::RobotState copy(_trj.getFirstWayPoint());
//   _trj.setRobotTrajectoryMsg(copy, joint_trajectory);

//   return true;
// }

// gsplines::GSpline
// interpolate_robot_trajectory(const moveit_msgs::RobotTrajectory &_msg,
//                              const gsplines::basis::Basis &_basis) {

//   return gsplines_ros::interpolate_joint_trajectory(_msg.joint_trajectory,
//                                                     _basis);
// }

std::vector<gsplines::GSpline>
forward_kinematics_frames(const gsplines::GSpline &joint_trj,
                          const moveit::core::JointModelGroup *group,
                          const moveit::core::RobotModelPtr &_model,
                          const std::vector<std::string> &_links) {

  // 1. Get best approximation of the given curve using gauss-lobatto points

  auto approx = gsplines::collocation::GaussLobattoLagrangeSpline::approximate(
      joint_trj, 12, 10);
  auto v = approx.get_domain_discretization();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      q_values = joint_trj(v);

  auto model = std::make_shared<moveit::core::RobotModel>(*_model);
  moveit::core::RobotState state(model);
  for (long i = 0; i < v.size(); i++) {
    state.setJointGroupPositions(group, q_values.data() + i * q_values.cols());
    state.updateLinkTransforms();
    for (const auto &link_name : _links) {
      state.getLinkModel(link_name);
    }
  }
  return {};
}

double get_max_frame_speed(const gsplines::GSpline &joint_trj,
                           const moveit::core::JointModelGroup *group,
                           const moveit::core::RobotModelPtr &_model,
                           const std::vector<std::string> &_links,
                           std::size_t nglp, std::size_t nintervals) {

  // 1. Get best approximation of the given curve using gauss-lobatto points

  auto approx = gsplines::collocation::GaussLobattoLagrangeSpline::approximate(
      joint_trj, nglp, nintervals);
  auto v = approx.get_domain_discretization();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      q_values = joint_trj(v);

  auto model = std::make_shared<moveit::core::RobotModel>(*_model);

  Eigen::MatrixXd jac;
  moveit::core::RobotState state(model);
  double max_speed = 0;

  for (long i = 0; i < v.size(); i++) {
    state.setJointGroupPositions(model->getJointModelGroup(group->getName()),
                                 q_values.data() + i * q_values.cols());
    double speed = 0;
    state.updateLinkTransforms();
    double max_frame_speed = 0;
    for (const auto &link_name : _links) {

      state.getJacobian(model->getJointModelGroup(group->getName()),
                        model->getLinkModel(link_name), Eigen::Vector3d::Zero(),
                        jac);

      speed = (jac.topRows(3) * q_values.row(i).transpose()).norm();
      if (speed > max_frame_speed) {
        max_frame_speed = speed;
      }
    }
    if (max_frame_speed > max_speed) {
      max_speed = max_frame_speed;
    }
  }
  return max_speed;
}

double get_max_frame_speed(const gsplines::GSpline &joint_trj,
                           const moveit::core::JointModelGroup *group,
                           const moveit::core::RobotModelConstPtr &_model,
                           double step) {

  auto time_values = Eigen::VectorXd::LinSpaced(static_cast<long>(1.0 / step),
                                                joint_trj.get_domain().first,
                                                joint_trj.get_domain().second);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      q_values = joint_trj(time_values);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      q_dot_values = joint_trj.derivate()(time_values);

  Eigen::MatrixXd jac;
  moveit::core::RobotState state(_model);
  double max_speed = 0;
  auto _links = group->getLinkModelNames();

  for (long i = 0; i < time_values.size(); i++) {
    state.setJointGroupPositions(group, q_values.data() + i * q_values.cols());
    double speed = 0;
    state.updateLinkTransforms();
    double max_frame_speed = 0;
    for (const auto &link_name : _links) {

      state.getJacobian(group, _model->getLinkModel(link_name),
                        Eigen::Vector3d::Zero(), jac);

      speed = (jac.topRows(3) * q_dot_values.row(i).transpose()).norm();

      if (speed > max_frame_speed) {
        max_frame_speed = speed;
      }
    }
    if (max_frame_speed > max_speed) {
      max_speed = max_frame_speed;
    }
  }
  return max_speed;
}

struct _boundVector {
  std::vector<double> acceleration_bounds;
  std::vector<double> velocity_bounds;
};
std::string vectorToString(const std::vector<double> &v) {
  std::array<char, 20> buff = {0};
  std::ostringstream oss;
  for (const auto &val : v) {
    std::snprintf(buff.data(), 20, // NOLINT
                  "%05.2fl", val);
    oss << buff.data() << " ";
  }
  return oss.str();
}
_boundVector _getBounds(const moveit::core::JointModelGroup &group,
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
      velocity_bounds.push_back(max_v * _vel_factor);
    } else {
      velocity_bounds.push_back(_vel_factor);
    }

    // set acceleration bounds
    if (bounds.acceleration_bounded_) {
      const double max_v = std::max(std::abs(bounds.max_acceleration_),
                                    std::abs(bounds.min_acceleration_));
      acceleration_bounds.push_back(max_v * _acc_factor);
    } else {
      acceleration_bounds.push_back(_acc_factor);
    }
  }
  return {std::move(velocity_bounds), std::move(acceleration_bounds)};
}

gsplines::GSpline
scale_trajectory(const gsplines::GSpline &trj,
                 const planning_interface::MotionPlanRequest &req,
                 planning_interface::MotionPlanResponse &res) {

  ROS_INFO_STREAM_NAMED(LOGNAME,
                        "Scaling trajectory: \n"
                        "  Velocity scalling factor = "
                            << req.max_acceleration_scaling_factor
                            << "\n Acceleration scaling factor "
                            << req.max_acceleration_scaling_factor); // NOLINT
  auto bounds =
      _getBounds(*res.trajectory_->getGroup(), req.max_velocity_scaling_factor,
                 req.max_acceleration_scaling_factor);
  ROS_INFO_STREAM_NAMED(
      LOGNAME, "Scaling trajectory: \n      velocity bounds "
                   << vectorToString(bounds.velocity_bounds)
                   << "\n     acceleration bounds "
                   << vectorToString(bounds.acceleration_bounds)); // NOLINT

  auto trj_1 =
      trj.linear_scaling_new_execution_time_max_velocity_max_acceleration(
          bounds.velocity_bounds, bounds.acceleration_bounds, 0.01);

  double max_group_frame_speed =
      get_max_frame_speed(trj_1, res.trajectory_->getGroup(),
                          res.trajectory_->getRobotModel(), 0.01);

  ROS_INFO_STREAM_NAMED(LOGNAME, "trajectory scaled to time execution "
                                     << trj_1.get_domain_length() << " [s]");

  ROS_INFO_STREAM_NAMED(LOGNAME, "The maximum velocity of the frames is "
                                     << max_group_frame_speed); // NOLINT

  ROS_INFO_STREAM_NAMED(LOGNAME, "Scaling trajectory: \n"
                                 "  Maximum cartesian speed = "
                                     << 0.3);
  if (max_group_frame_speed < 0.3) {
    return trj_1;
  }
  return trj_1;
  double t0 = trj_1.get_domain_length();
  return trj_1.linear_scaling_new_execution_time(t0 * max_group_frame_speed /
                                                 0.3);
}
} // namespace gsplines_moveit
