
#include <class_loader/class_loader.hpp>
#include <gsplines_moveit/MinimumJerkAdapter.hpp>

#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Optimization/ipopt_solver.hpp>
#include <gsplines_moveit/gsplines_moveit.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <moveit/robot_state/conversions.h>

namespace gsplines_moveit {

bool MinimumJerkAdapter::adaptAndPlan(
    const PlannerFn &planner,
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const planning_interface::MotionPlanRequest &req,
    planning_interface::MotionPlanResponse &res,
    std::vector<std::size_t> &) const {

  bool result = planner(planning_scene, req, res);
  if (result && res.trajectory_) {
    result = compute_minimum_jerk_trajectory(
        *res.trajectory_, ros::Duration(0.01), req.max_velocity_scaling_factor,
        req.max_acceleration_scaling_factor);
    return result;
  }

  return true;
}

std::string MinimumJerkAdapter::getDescription() const {
  return "Minimizes the jerk";
}
void MinimumJerkAdapter::initialize(const ros::NodeHandle &) {};
} // namespace gsplines_moveit
CLASS_LOADER_REGISTER_CLASS(gsplines_moveit::MinimumJerkAdapter,
                            planning_request_adapter::PlanningRequestAdapter);
