
#include <class_loader/class_loader.hpp>
#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Optimization/ipopt_solver.hpp>
#include <gsplines_moveit/MinimumSobolevSeminormAdapter.hpp>
#include <gsplines_moveit/gsplines_moveit.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <moveit/robot_state/conversions.h>

namespace gsplines_moveit {
class MinimumSobolevSeminormAdapter::Impl {};

MinimumSobolevSeminormAdapter::~MinimumSobolevSeminormAdapter() = default;

MinimumSobolevSeminormAdapter::MinimumSobolevSeminormAdapter() = default;

bool MinimumSobolevSeminormAdapter::adaptAndPlan(
    const PlannerFn &planner,
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const planning_interface::MotionPlanRequest &req,
    planning_interface::MotionPlanResponse &res,
    std::vector<std::size_t> &) const {

  bool result = planner(planning_scene, req, res);
  if (result && res.trajectory_) {

    const moveit::core::JointModelGroup *group = res.trajectory_->getGroup();
    const moveit::core::RobotModel &rmodel = group->getParentModel();
    // const std::vector<int> &idx = group->getVariableIndexList();
    const std::vector<std::string> &vars = group->getVariableNames();

    std::vector<moveit::core::VariableBounds> bounds;
    std::vector<double> velocity_bounds;
    std::vector<double> acceleration_bounds;
    std::transform(vars.begin(), vars.end(),
                   std::back_inserter(velocity_bounds),
                   [rmodel](const std::string &_var_name) {
                     return rmodel.getVariableBounds(_var_name).max_velocity_;
                   });

    std::transform(
        vars.begin(), vars.end(), std::back_inserter(acceleration_bounds),
        [rmodel](const std::string &_var_name) {
          return rmodel.getVariableBounds(_var_name).max_acceleration_;
        });
    Eigen::MatrixXd waypoints = robot_trajectory_waypoints(*res.trajectory_);

    gsplines::GSpline gspline = gsplines::optimization::optimal_sobolev_norm(
        waypoints, gsplines::basis::BasisLegendre(6), {{4, 1.0}}, 10.0);

    trajectory_msgs::JointTrajectory trj =
        gsplines_ros::gspline_to_joint_trajectory_msg(
            gspline, req.start_state.joint_state.name, ros::Duration(0.01));

    res.trajectory_->setRobotTrajectoryMsg(res.trajectory_->getFirstWayPoint(),
                                           trj);

    return result;
  }

  return true;
}

std::string MinimumSobolevSeminormAdapter::getDescription() const {
  return "Minimizes the desired sobolev seminorm";
}
void MinimumSobolevSeminormAdapter::initialize(
    const ros::NodeHandle &node_handle) {};
} // namespace gsplines_moveit
CLASS_LOADER_REGISTER_CLASS(gsplines_moveit::MinimumSobolevSeminormAdapter,
                            planning_request_adapter::PlanningRequestAdapter);
