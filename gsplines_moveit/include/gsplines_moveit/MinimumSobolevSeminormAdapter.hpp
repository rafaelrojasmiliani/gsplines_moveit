#pragma once
#include <moveit/planning_request_adapter/planning_request_adapter.h>
namespace gsplines_moveit {

class MinimumSobolevSeminormAdapter
    : public ::planning_request_adapter::PlanningRequestAdapter {
public:
  explicit MinimumSobolevSeminormAdapter();
  /// Interface
  bool adaptAndPlan(const PlannerFn &planner,
                    const planning_scene::PlanningSceneConstPtr &planning_scene,
                    const planning_interface::MotionPlanRequest &req,
                    planning_interface::MotionPlanResponse &res,
                    std::vector<std::size_t> &added_path_index) const override;

  /// Returns string description
  std::string getDescription() const override;

  /// Initialization
  void initialize(const ros::NodeHandle &node_handle) override;

  ~MinimumSobolevSeminormAdapter() override;

protected:
  class Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gsplines_moveit
