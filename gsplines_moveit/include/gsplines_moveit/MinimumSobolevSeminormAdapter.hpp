#include <class_loader/class_loader.hpp>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#ifndef MINIMUMSOBOLEVSEMINORMADAPTER_H
#define MINIMUMSOBOLEVSEMINORMADAPTER_H

namespace gsplines_moveit {

class MinimumSobolevSeminormAdapter
    : public ::planning_request_adapter::PlanningRequestAdapter {
public:
  MinimumSobolevSeminormAdapter();

  virtual bool
  adaptAndPlan(const PlannerFn &planner,
               const planning_scene::PlanningSceneConstPtr &planning_scene,
               const planning_interface::MotionPlanRequest &req,
               planning_interface::MotionPlanResponse &res,
               std::vector<std::size_t> &added_path_index) const override;

  virtual std::string getDescription() const override;
  virtual void initialize(const ros::NodeHandle &node_handle) override;
};

} // namespace gsplines_moveit
#endif /* MINIMUMSOBOLEVSEMINORMADAPTER_H */
