
#include <gsplines_moveit/MinimumSobolevSeminormAdapter.hpp>

namespace gsplines_moveit {

MinimumSobolevSeminormAdapter::MinimumSobolevSeminormAdapter()
    : ::planning_request_adapter::PlanningRequestAdapter() {}

bool MinimumSobolevSeminormAdapter::adaptAndPlan(
    const PlannerFn &planner,
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const planning_interface::MotionPlanRequest &req,
    planning_interface::MotionPlanResponse &res,
    std::vector<std::size_t> &added_path_index) const {
  /*
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_) {

      return result;
    }
    */
  return true;
}

std::string MinimumSobolevSeminormAdapter::getDescription() const {
  return "Minimizes the desired sobolev seminorm";
}
void MinimumSobolevSeminormAdapter::initialize(
    const ros::NodeHandle &node_handle){};
} // namespace gsplines_moveit
CLASS_LOADER_REGISTER_CLASS(gsplines_moveit::MinimumSobolevSeminormAdapter,
                            planning_request_adapter::PlanningRequestAdapter);
