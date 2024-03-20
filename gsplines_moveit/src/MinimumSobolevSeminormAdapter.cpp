
#include <class_loader/class_loader.hpp>
#include <dynamic_reconfigure/server.h>
#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Optimization/ipopt_solver.hpp>
#include <gsplines_moveit/MinimumSobolevSeminormAdapter.hpp>
#include <gsplines_moveit/MinimumSobolevSeminormDynamicReconfigureConfig.h>
#include <gsplines_moveit/gsplines_moveit.hpp>
#include <gsplines_msgs/GetBasis.h>
#include <gsplines_ros/gsplines_ros.hpp>
#include <moveit/robot_state/conversions.h>
#include <optional>
#include <rosconsole/macros_generated.h>

namespace gsplines_moveit {

struct _boundVector {
  std::vector<double> acceleration_bounds;
  std::vector<double> velocity_bounds;
};

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

static const std::string LOGNAME = "minimum_sobolev_norm_adapter";

class MinimumSobolevSeminormAdapter::Impl {
public:
  using GetBasisSrv = gsplines_msgs::GetBasis;
  std::unique_ptr<gsplines::basis::Basis> basis;
  std::vector<std::pair<std::size_t, double>> weights;

  ProblemType problem_type_;

  double k_;
  std::size_t polynomial_degree_;
  std::function<double(int)> exec_time;

public:
  /// convenience type definition
  using ConfigType =
      gsplines_moveit::MinimumSobolevSeminormDynamicReconfigureConfig;

  /// dynamic reconfigure server
  dynamic_reconfigure::Server<ConfigType> server_;

  ros::ServiceServer get_basis_server_;
  ros::NodeHandle nh_;

  Impl() : server_(ros::NodeHandle("~/gsplines_moveit")) {

    server_.setCallback([this](ConfigType &_cfg, uint32_t _level) {
      this->set_parameters(_cfg, _level);
    });

    get_basis_server_ = nh_.advertiseService(
        "gsplines_moveit/get_basis",
        &MinimumSobolevSeminormAdapter::Impl::get_basis, this);
  }

  bool get_basis(typename GetBasisSrv::Request &req,
                 typename GetBasisSrv::Response &res) {

    (void)req;
    res.basis = gsplines_ros::basis_to_basis_msg(*this->basis);
    res.success = true;
    res.message = "";
    return true;
  }

  void set_parameters(ConfigType &_cfg, uint32_t /*level*/) {

    polynomial_degree_ = _cfg.PolynomialDegree;

    if (_cfg.Basis == 0) {
      basis =
          std::make_unique<gsplines::basis::BasisLegendre>(polynomial_degree_);
    }

    weights.clear();
    switch (_cfg.OptimizationType) {
    case 0:
      this->problem_type_ = ProblemType::MinimumVelocity;
      basis =
          std::make_unique<gsplines::basis::BasisLegendre>(polynomial_degree_);
      weights.emplace_back(std::make_pair(1, 1.0));
      exec_time = [](int wpn) -> double { return wpn - 1; };
      break;
    case 1:
      this->problem_type_ = ProblemType::MinimumAcceleration;
      basis = std::make_unique<gsplines::basis::BasisLegendre>(4);
      weights.emplace_back(std::make_pair(2, 1.0));
      exec_time = [](int wpn) -> double { return wpn - 1; };
      break;
    case 2:
      this->problem_type_ = ProblemType::MinimumJerk;
      basis = std::make_unique<gsplines::basis::BasisLegendre>(6);
      weights.emplace_back(std::make_pair(3, 1.0));
      exec_time = [](int wpn) -> double { return wpn - 1; };
      break;
    case 3: {
      k_ = _cfg.kFactor;
      double k4 = std::pow(k_, 4);
      // execution_time =
      double alpha = k4 / (1.0 + k4);
      basis = std::make_unique<gsplines::basis::Basis0101>(alpha);
      weights.emplace_back(std::make_pair(1, alpha));
      weights.emplace_back(std::make_pair(3, 1.0 - alpha));
      exec_time = [](int numberOfWaypoints) -> double {
        return 4.0 * float(numberOfWaypoints - 1) / std::sqrt(2.0);
      };
      break;
    }
    case 4:
      this->problem_type_ = ProblemType::Custom;
      weights.clear();
      weights.push_back(std::make_pair(1, _cfg.weight1));
      weights.push_back(std::make_pair(2, _cfg.weight2));
      weights.push_back(std::make_pair(3, _cfg.weight3));
      weights.push_back(std::make_pair(4, _cfg.weight4));
      weights.push_back(std::make_pair(5, _cfg.weight5));
      break;
    default:
      throw std::runtime_error("Undefined OptimizationType");
    }
  }
};

MinimumSobolevSeminormAdapter::~MinimumSobolevSeminormAdapter() = default;

MinimumSobolevSeminormAdapter::MinimumSobolevSeminormAdapter()
    : ::planning_request_adapter::PlanningRequestAdapter(),
      m_impl(std::make_unique<MinimumSobolevSeminormAdapter::Impl>()) {}

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
    std::vector<std::size_t> &) const {

  bool result = planner(planning_scene, req, res);

  if (result && res.trajectory_) {

    ROS_INFO_NAMED(LOGNAME, "Starting optimization");
    ROS_INFO_NAMED(LOGNAME, "Basis %s", m_impl->basis->get_name().c_str());
    ROS_INFO_NAMED(LOGNAME, "Problem type %s",
                   problemTypeToString(m_impl->problem_type_).c_str());
    ROS_INFO_NAMED(LOGNAME, "Number of waypoint: %zu",
                   res.trajectory_->getWayPointCount());

    ///--------------------
    // result = compute_minimum_sobolev_semi_norm_robot_trajectory(
    //     *res.trajectory_, *m_impl->basis, m_impl->weights,
    //     ros::Duration(0.01), req.max_velocity_scaling_factor,
    //     req.max_acceleration_scaling_factor,
    //     m_impl->exec_time(res.trajectory_->getWayPointCount()));

    Eigen::MatrixXd waypoints = robot_trajectory_waypoints(*res.trajectory_);

    gsplines::GSpline trj = gsplines::optimization::optimal_sobolev_norm(
        waypoints, *m_impl->basis, m_impl->weights,
        m_impl->exec_time(res.trajectory_->getWayPointCount()));

    auto bounds = _getBounds(*res.trajectory_->getGroup(),
                             req.max_velocity_scaling_factor,
                             req.max_acceleration_scaling_factor);
    auto trj2 =
        trj.linear_scaling_new_execution_time_max_velocity_max_acceleration(
            bounds.velocity_bounds, bounds.acceleration_bounds, 0.01);

    // workarround to handle basis0101, which change when time-scaled.
    m_impl->basis = trj2.get_basis().clone();

    std::string s;
    trajectory_msgs::JointTrajectory trj_msg;
    if (m_impl->nh_.hasParam("moveit_controller_manager") &&
        m_impl->nh_.getParam("moveit_controller_manager", s) &&
        s == "gsplines_moveit/GSplinesControllerManager") {
      trj_msg = gsplines_ros::gspline_to_minimal_joint_trajectory_msg(
          trj2, res.trajectory_->getGroup()->getVariableNames());
    } else {
      trj_msg = gsplines_ros::function_to_joint_trajectory_msg(
          trj2, res.trajectory_->getGroup()->getVariableNames(),
          ros::Duration(0.01));
    }
    ///--------------------
    ROS_INFO_NAMED(LOGNAME, "Optimization finished");
    return result;
  }

  return false;
}

std::string MinimumSobolevSeminormAdapter::getDescription() const {
  return "Minimizes the desired sobolev seminorm";
}
void MinimumSobolevSeminormAdapter::initialize(const ros::NodeHandle &) {};
} // namespace gsplines_moveit
CLASS_LOADER_REGISTER_CLASS(gsplines_moveit::MinimumSobolevSeminormAdapter,
                            planning_request_adapter::PlanningRequestAdapter);
