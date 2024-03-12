#pragma once
#include <gsplines_moveit/MinimumSobolevSeminormAdapter.hpp>

namespace gsplines_moveit {

class MinimumJerkAdapter : public MinimumSobolevSeminormAdapter {
private:
public:
  bool adaptAndPlan(const PlannerFn &planner,
                    const planning_scene::PlanningSceneConstPtr &planning_scene,
                    const planning_interface::MotionPlanRequest &req,
                    planning_interface::MotionPlanResponse &res,
                    std::vector<std::size_t> &added_path_index) const override;

  std::string getDescription() const override;
  void initialize(const ros::NodeHandle &node_handle) override;
};

} // namespace gsplines_moveit
