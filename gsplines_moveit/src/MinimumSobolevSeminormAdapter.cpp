
#include <class_loader/class_loader.hpp> //NOLINT

#include <dynamic_reconfigure/server.h>

#include <gsplines/Basis/Basis.hpp>
#include <gsplines/Basis/Basis0101.hpp>
#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/GSpline.hpp>
#include <gsplines/Optimization/ipopt_solver.hpp>

#include <gsplines_moveit/MinimumSobolevSeminormAdapter.hpp>
#include <gsplines_moveit/MinimumSobolevSeminormDynamicReconfigureConfig.h>
#include <gsplines_moveit/gsplines_moveit.hpp>

#include <gsplines_msgs/GetBasis.h>

#include <gsplines_ros/gsplines_ros.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_model/joint_model_group.h>

#include <rosconsole/macros_generated.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
namespace gsplines_moveit {

struct _boundVector {
  std::vector<double> acceleration_bounds;
  std::vector<double> velocity_bounds;
};

std::string vectorToString(const std::vector<double> &v) {
  std::array<char, 20> buff = {0};
  std::ostringstream oss;
  for (const auto &val : v) {
    std::snprintf(buff.data(), 20, //
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

static const std::string LOGNAME = "minimum_sobolev_norm_adapter";

class MinimumSobolevSeminormAdapter::Impl {
public:
  using GetBasisSrv = gsplines_msgs::GetBasis;
  std::unique_ptr<gsplines::basis::Basis> basis;
  std::vector<std::pair<std::size_t, double>> weights;

  ProblemType problem_type_ = ProblemType::MinimumVelocity;

  double k_{};
  std::size_t polynomial_degree_{};
  std::function<double(int)> exec_time;

  /// convenience type definition
  using ConfigType =
      gsplines_moveit::MinimumSobolevSeminormDynamicReconfigureConfig;

  /// dynamic reconfigure server
  dynamic_reconfigure::Server<ConfigType> server_;

  ros::ServiceServer get_basis_server_;
  ros::NodeHandle nh_prv_{"~"};

  Impl() : server_(ros::NodeHandle("~/gsplines_moveit")) {

    server_.setCallback([this](ConfigType &_cfg, uint32_t _level) {
      this->set_parameters(_cfg, _level);
    });

    get_basis_server_ = nh_prv_.advertiseService(
        "gsplines_moveit/get_basis",
        &MinimumSobolevSeminormAdapter::Impl::get_basis, this);
  }

  bool get_basis(typename GetBasisSrv::Request &req, // NOLINT
                 typename GetBasisSrv::Response &res) {

    (void)req;
    res.basis = gsplines_ros::basis_to_basis_msg(*this->basis);
    res.success = true; // NOLINT
    res.message = "";
    return true;
  }

  void set_parameters(ConfigType &_cfg, uint32_t /*level*/) {

    polynomial_degree_ = _cfg.PolynomialDegree;

    if (_cfg.Basis == 0) {
      basis = std::make_unique<gsplines::basis::BasisLegendre>(
          polynomial_degree_ + 1);
    }

    weights.clear();
    switch (_cfg.OptimizationType) {
    case 0:
      this->problem_type_ = ProblemType::MinimumVelocity;
      basis = std::make_unique<gsplines::basis::BasisLegendre>(
          polynomial_degree_ + 1);
      weights.emplace_back(1, 1.0);
      exec_time = [](int wpn) -> double { return wpn - 1; };
      break;
    case 1:
      this->problem_type_ = ProblemType::MinimumAcceleration;
      basis = std::make_unique<gsplines::basis::BasisLegendre>(4);
      weights.emplace_back(2, 1.0);
      exec_time = [](int wpn) -> double { return wpn - 1; };
      break;
    case 2:
      this->problem_type_ = ProblemType::MinimumJerk;
      basis = std::make_unique<gsplines::basis::BasisLegendre>(6);
      weights.emplace_back(3, 1.0);
      exec_time = [](int wpn) -> double { return wpn - 1; };
      break;
    case 3: {
      k_ = _cfg.kFactor;
      const double k4 = std::pow(k_, 4);
      // execution_time =
      const double alpha = k4 / (1.0 + k4);
      basis = std::make_unique<gsplines::basis::Basis0101>(alpha);
      weights.emplace_back(1, alpha);
      weights.emplace_back(3, 1.0 - alpha);
      exec_time = [](int numberOfWaypoints) -> double {
        return 4.0 * float(numberOfWaypoints - 1) / std::sqrt(2.0);
      };
      break;
    }
    case 4:
      this->problem_type_ = ProblemType::Custom;
      weights.clear();
      weights.emplace_back(1, _cfg.weight1);
      weights.emplace_back(2, _cfg.weight2);
      weights.emplace_back(3, _cfg.weight3);
      weights.emplace_back(4, _cfg.weight4);
      weights.emplace_back(5, _cfg.weight5);
      break;
    default:
      throw std::runtime_error("Undefined OptimizationType");
    }
  }
};

MinimumSobolevSeminormAdapter::~MinimumSobolevSeminormAdapter() = default;

MinimumSobolevSeminormAdapter::MinimumSobolevSeminormAdapter()
    : m_impl(std::make_unique<MinimumSobolevSeminormAdapter::Impl>()) {}

std::string
MinimumSobolevSeminormAdapter::problemTypeToString(const ProblemType &_type) {

  switch (_type) {
  case ProblemType::MinimumVelocity:
    return "MinimumVelocity";
  case ProblemType::MinimumAcceleration:
    return "MinimumAcceleration";
  case ProblemType::MinimumJerk:
    return "MinimumJerk";
  case ProblemType::Rojas:
    return "Rojas";
  case ProblemType::Custom:
    return "Custom";
  }
  return "MinimumVelocity";
}

bool MinimumSobolevSeminormAdapter::adaptAndPlan(
    const PlannerFn &planner,
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const planning_interface::MotionPlanRequest &req,
    planning_interface::MotionPlanResponse &res,
    std::vector<std::size_t> &added_path_index) const {

  (void)added_path_index; // unused parameter
  const bool result = planner(planning_scene, req, res);

  if (result && res.trajectory_) {

    ROS_INFO_STREAM_NAMED(LOGNAME, "Starting optimization");         // NOLINT
    ROS_INFO_STREAM_NAMED(LOGNAME,                                   // NOLINT
                          "Basis " << m_impl->basis->get_name());    // NOLINT
    ROS_INFO_STREAM_NAMED(                                           // NOLINT
        LOGNAME, "Problem type "                                     // NOLINT
                     << problemTypeToString(m_impl->problem_type_)); // NOLINT
    ROS_INFO_STREAM_NAMED(                                           // NOLINT
        LOGNAME, "Number of waypoint: "                              // NOLINT
                     << res.trajectory_->getWayPointCount());        // NOLINT

    ///--------------------
    // result = compute_minimum_sobolev_semi_norm_robot_trajectory(
    //     *res.trajectory_, *m_impl->basis, m_impl->weights,
    //     ros::Duration(0.01), req.max_velocity_scaling_factor,
    //     req.max_acceleration_scaling_factor,
    //     m_impl->exec_time(res.trajectory_->getWayPointCount()));

    const Eigen::MatrixXd waypoints =
        robot_trajectory_waypoints(*res.trajectory_);

    const gsplines::GSpline trj = gsplines::optimization::optimal_sobolev_norm(
        waypoints, *m_impl->basis, m_impl->weights,
        m_impl->exec_time(
            static_cast<int>(res.trajectory_->getWayPointCount())));
    ROS_INFO_STREAM_NAMED(LOGNAME, "Optimization finished"); // NOLINT

    ROS_INFO_STREAM_NAMED(LOGNAME,
                          "Scaling trajectory: \n"
                          "  Velocity scalling factor = "
                              << req.max_acceleration_scaling_factor
                              << "\n Acceleration scaling factor "
                              << req.max_acceleration_scaling_factor); // NOLINT
    auto bounds = _getBounds(*res.trajectory_->getGroup(),
                             req.max_velocity_scaling_factor,
                             req.max_acceleration_scaling_factor);
    ROS_INFO_STREAM_NAMED(
        LOGNAME, "Scaling trajectory: \n      velocity bounds "
                     << vectorToString(bounds.velocity_bounds)
                     << "\n     acceleration bounds "
                     << vectorToString(bounds.acceleration_bounds)); // NOLINT
    auto trj2 =
        trj.linear_scaling_new_execution_time_max_velocity_max_acceleration(
            bounds.velocity_bounds, bounds.acceleration_bounds, 0.01);
    ROS_INFO_STREAM_NAMED(LOGNAME, "Scaling trajectory: ok"); // NOLINT

    // workarround to handle basis0101, which change when time-scaled.
    m_impl->basis = trj2.get_basis().clone();

    std::string s;
    trajectory_msgs::JointTrajectory trj_msg;
    if (m_impl->nh_prv_.hasParam("moveit_controller_manager") &&
        m_impl->nh_prv_.getParam("moveit_controller_manager", s) &&
        s == "gsplines_moveit/GSplinesControllerManager") {
      ROS_INFO_STREAM_NAMED(LOGNAME, // NOLINT
                            "Generating minimum trajectory message to be "
                            "acquired by GSplinesControllerManager"); // NOLINT
      trj_msg = gsplines_ros::gspline_to_minimal_joint_trajectory_msg(
          trj2, res.trajectory_->getGroup()->getVariableNames());

    } else {
      ROS_INFO_STREAM_NAMED(LOGNAME, "Generating Trajectory message"); // NOLINT
      trj_msg = gsplines_ros::function_to_joint_trajectory_msg(
          trj2, res.trajectory_->getGroup()->getVariableNames(),
          ros::Duration(0.01));
    }
    if ((trj_msg.header.stamp + trj_msg.points.back().time_from_start)
            .isZero()) {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Zero time trj"); // NOLINT
    }
    const moveit::core::RobotState copy(res.trajectory_->getFirstWayPoint());
    res.trajectory_->setRobotTrajectoryMsg(copy, trj_msg);
    ROS_INFO_STREAM_NAMED(LOGNAME,                              // NOLINT
                          "Generating Trajectory message: ok"); // NOLINT
    ///--------------------
    return result;
  }

  return false;
}

std::string MinimumSobolevSeminormAdapter::getDescription() const {
  return "Minimizes the desired sobolev seminorm";
}
void MinimumSobolevSeminormAdapter::initialize(const ros::NodeHandle & /*nh*/) {
};
} // namespace gsplines_moveit
CLASS_LOADER_REGISTER_CLASS(                           // NOLINT
    gsplines_moveit::MinimumSobolevSeminormAdapter,    // NOLINT
    planning_request_adapter::PlanningRequestAdapter); // NOLINT
