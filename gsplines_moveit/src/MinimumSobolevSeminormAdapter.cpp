
#include <class_loader/class_loader.hpp>
#include <dynamic_reconfigure/server.h>
#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Optimization/ipopt_solver.hpp>
#include <gsplines_moveit/MinimumSobolevSeminormAdapter.hpp>
#include <gsplines_moveit/MinimumSobolevSeminormDynamicReconfigureConfig.h>
#include <gsplines_moveit/gsplines_moveit.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <moveit/robot_state/conversions.h>
#include <rosconsole/macros_generated.h>

namespace gsplines_moveit {

static const std::string LOGNAME = "minimum_sobolev_norm_adapter";
class MinimumSobolevSeminormAdapter::Impl {
public:
  struct CustomParameters {

    std::unique_ptr<gsplines::basis::Basis> basis_;
    std::vector<std::pair<std::size_t, double>> weights;
  };

  ProblemType problem_type_;

  double k_;
  std::size_t polynomial_degree_;

public:
  using ConfigType =
      gsplines_moveit::MinimumSobolevSeminormDynamicReconfigureConfig;
  dynamic_reconfigure::Server<ConfigType> server_;
  Impl() : server_(ros::NodeHandle("~/gsplines_moveit")) {

    server_.setCallback([this](ConfigType &_cfg, uint32_t _level) {
      this->set_parameters(_cfg, _level);
    });
  }

  void set_parameters(ConfigType &_cfg, uint32_t _level) {

    // Parameter 1 changed
    if (_level == 0) { // Check if bit 0 is set
      switch (_cfg.OptimizationType) {
      case 0:
        this->problem_type_ = ProblemType::MinimumVelocity;
        break;
      case 1:
        this->problem_type_ = ProblemType::MinimumAcceleration;
        break;
      case 2:
        this->problem_type_ = ProblemType::MinimumJerk;
        break;
      case 3:
        this->problem_type_ = ProblemType::Rojas;
        break;
      case 4:
        this->problem_type_ = ProblemType::Custom;
        break;
      default:
        throw std::runtime_error("Undefined OptimizationType");
      }
    }

    if (_level == 1) {
      polynomial_degree_ = _cfg.PolynomialDegree;
    }

    if (_level == 2) {
      k_ = _cfg.kFactor;
    }

    if (_level == 3) {
      k_ = _cfg.weight1;
    }
    if (_level == 4) {
      k_ = _cfg.weight2;
    }
    if (_level == 5) {
      k_ = _cfg.weight3;
    }
    if (_level == 6) {
      k_ = _cfg.weight4;
    }
    if (_level == 7) {
      k_ = _cfg.weight5;
    }
    if (_level == 8) {
      ROS_ERROR("set basis");
    }
  }
  ~Impl() {}
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

  ROS_ERROR("a");
  bool result = planner(planning_scene, req, res);

  ROS_ERROR("b");
  std::unique_ptr<gsplines::basis::Basis> basis;
  std::vector<std::pair<std::size_t, double>> weights;
  if (result && res.trajectory_) {

    switch (m_impl->problem_type_) {
    case ProblemType::MinimumVelocity:
      basis = std::make_unique<gsplines::basis::BasisLegendre>(6);
      weights.emplace_back(std::make_pair(1, 1.0));
      break;
    case ProblemType::MinimumAcceleration:
      basis = std::make_unique<gsplines::basis::BasisLegendre>(4);
      weights.emplace_back(std::make_pair(2, 1.0));
      break;
    case ProblemType::MinimumJerk:
      basis = std::make_unique<gsplines::basis::BasisLegendre>(6);
      weights.emplace_back(std::make_pair(3, 1.0));
      break;
    case ProblemType::Rojas: {
      basis = std::make_unique<gsplines::basis::BasisLegendre>(6);
      double k4 = std::pow(m_impl->k_, 4);
      // execution_time = 4.0*float(N)/np.sqrt(2.0)
      double alpha = k4 / (1.0 + k4);
      weights.emplace_back(std::make_pair(1, alpha));
      weights.emplace_back(std::make_pair(3, 1.0 - alpha));
      break;
      ;
    }
    case ProblemType::Custom:
      ROS_ERROR("Not implemented %d", static_cast<int>(m_impl->problem_type_));
      res.error_code_ = moveit::core::MoveItErrorCode::PLANNING_FAILED;
      return false;
    default:
      ROS_ERROR("Unknow problem type %d",
                static_cast<int>(m_impl->problem_type_));
      res.error_code_ = moveit::core::MoveItErrorCode::PLANNING_FAILED;
      return false;
    }

    ROS_INFO_NAMED(LOGNAME, "Starting optimization");
    ROS_INFO_NAMED(LOGNAME, "Basis %s", basis->get_name().c_str());
    ROS_INFO_NAMED(LOGNAME, "Problem type %s",
                   problemTypeToString(m_impl->problem_type_).c_str());

    result = compute_minimum_sobolev_semi_norm_robot_trajectory(
        *res.trajectory_, *basis, weights, ros::Duration(0.01),
        req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor);
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
