
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
  using ConfigType =
      gsplines_moveit::MinimumSobolevSeminormDynamicReconfigureConfig;
  dynamic_reconfigure::Server<ConfigType> server_;
  Impl() {

    ROS_WARN_NAMED(LOGNAME, "wwweeeee++++");
    server_.setCallback([](ConfigType &_cfg, uint32_t level_) {
      (void)level_;

      ROS_WARN("wwweeeee");
      _cfg.sobol_degree = 4;
    });
  }
};

MinimumSobolevSeminormAdapter::~MinimumSobolevSeminormAdapter() = default;

MinimumSobolevSeminormAdapter::MinimumSobolevSeminormAdapter()
    : ::planning_request_adapter::PlanningRequestAdapter(),
      m_impl(std::make_unique<MinimumSobolevSeminormAdapter::Impl>()) {}

bool MinimumSobolevSeminormAdapter::adaptAndPlan(
    const PlannerFn &planner,
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const planning_interface::MotionPlanRequest &req,
    planning_interface::MotionPlanResponse &res,
    std::vector<std::size_t> &) const {

  bool result = planner(planning_scene, req, res);
  if (result && res.trajectory_) {

    ROS_INFO_NAMED(LOGNAME, "Starting optimization");
    result = compute_minimum_sobolev_semi_norm_robot_trajectory(
        *res.trajectory_, gsplines::basis::BasisLegendre(8), {{4, 1.0}},
        ros::Duration(0.01), req.max_velocity_scaling_factor,
        req.max_acceleration_scaling_factor);
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
