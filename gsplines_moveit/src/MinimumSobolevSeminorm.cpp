
#include <class_loader/class_loader.hpp>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
namespace gsplines_moveit {

class MinimumSobolevSeminormAdapter
    : public planning_request_adapter::PlanningRequestAdapter {
private:
public:
  MinimumSobolevSeminormAdapter()
      : planning_request_adapter::PlanningRequestAdapter(){};

  bool adaptAndPlan(const PlannerFn &planner,
                    const planning_scene::PlanningSceneConstPtr &planning_scene,
                    const planning_interface::MotionPlanRequest &req,
                    planning_interface::MotionPlanResponse &res,
                    std::vector<std::size_t> &added_path_index) const override {
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_) {

      return result;
    }
    return true;
  }

  std::string getDescription() const override {
    return "Minimizes the desired sobolev seminorm";
  }
  void initialize(const ros::NodeHandle &node_handle) override{};
};

} // namespace gsplines_moveit
CLASS_LOADER_REGISTER_CLASS(gsplines_moveit::MinimumSobolevSeminormAdapter,
                            planning_request_adapter::PlanningRequestAdapter);
