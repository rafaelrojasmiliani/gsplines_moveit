
#include <gsplines/Basis/Basis.hpp>
#include <gsplines/GSpline.hpp>

#include <gsplines_moveit/GSplinesControllerHandler.hpp>

#include <gsplines_msgs/FollowJointGSplineActionFeedback.h>
#include <gsplines_msgs/FollowJointGSplineActionGoal.h>
#include <gsplines_msgs/FollowJointGSplineActionResult.h>
#include <gsplines_msgs/GetBasis.h>

#include <ros/console.h>

#include <gsplines_ros/gsplines_ros.hpp>

#include <moveit/utils/xmlrpc_casts.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <string>

static const std::string LOGNAME("GSplinesControllerHandler");

namespace gsplines_moveit {

bool FollowJointGSplineControllerHandle::sendTrajectory(
    const moveit_msgs::RobotTrajectory &trajectory) {
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "new trajectory to " << name_);

  if (!controller_action_client_) {
    return false;
  }

  if (!trajectory.multi_dof_joint_trajectory.points.empty()) {
    ROS_WARN_STREAM_NAMED(LOGNAME,                                    // NOLINT
                          "%s cannot execute multi-dof trajectories." // NOLINT
                              << name_);                              // NOLINT
  }

  if (done_) {
    ROS_DEBUG_STREAM_NAMED(LOGNAME,                            // NOLINT
                           "sending trajectory to " << name_); // NOLINT
  } else {
    ROS_DEBUG_STREAM_NAMED( // NOLINT
        LOGNAME,            // NOLINT
        "sending continuation for the currently executed trajectory to " // NOLINT
            << name_); // NOLINT
  }
  gsplines_msgs::GetBasis getBasisCall;
  if (!get_basis_.exists()) {
    ROS_DEBUG_STREAM_NAMED(LOGNAME,
                           "get basis service does not exists"); // NOLINT
    return false;
  }
  if (!get_basis_.call(getBasisCall)) {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "could not retrieve basis"); // NOLINT
    return false;
  }

  std::shared_ptr<gsplines::basis::Basis> basis =
      gsplines_ros::basis_msg_to_basis(getBasisCall.response.basis);

  auto gspline = gsplines_ros::interpolate_joint_trajectory(
      trajectory.joint_trajectory, *basis);

  gsplines_msgs::FollowJointGSplineGoal goal;
  goal = goal_template_;
  goal.gspline = gsplines_ros::gspline_to_joint_gspline_msg(
      gspline, trajectory.joint_trajectory.joint_names);
  controller_action_client_->sendGoal(
      goal,
      [this](const auto &state, const auto &result) {
        controllerDoneCallback(state, result);
      },
      [this] { controllerActiveCallback(); },
      [this](const auto &feedback) { controllerFeedbackCallback(feedback); });
  done_ = false;
  last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
  return true;
}

void FollowJointGSplineControllerHandle::configure(
    XmlRpc::XmlRpcValue &config) {
  /// Get parameters from the parameter server and set them into goal_template_.
  if (config.hasMember("path_tolerance")) {
    configure(config["path_tolerance"], "path_tolerance",
              goal_template_.path_tolerance);
  }
  if (config.hasMember("goal_tolerance")) {
    configure(config["goal_tolerance"], "goal_tolerance",
              goal_template_.goal_tolerance);
  }
  if (config.hasMember("goal_time_tolerance")) {
    goal_template_.goal_time_tolerance =
        ros::Duration(moveit::core::parseDouble(config["goal_time_tolerance"]));
  }
}

namespace {
enum ToleranceVariables { POSITION, VELOCITY, ACCELERATION };
template <ToleranceVariables>
double &variable(control_msgs::JointTolerance &msg);

template <>
inline double &variable<POSITION>(control_msgs::JointTolerance &msg) {
  return msg.position;
}
template <>
inline double &variable<VELOCITY>(control_msgs::JointTolerance &msg) {
  return msg.velocity;
}
template <>
inline double &variable<ACCELERATION>(control_msgs::JointTolerance &msg) {
  return msg.acceleration;
}

static std::map<ToleranceVariables, std::string> VAR_NAME = {
    {POSITION, "position"},
    {VELOCITY, "velocity"},
    {ACCELERATION, "acceleration"}};
static std::map<ToleranceVariables, decltype(&variable<POSITION>)> VAR_ACCESS =
    {{POSITION, &variable<POSITION>},
     {VELOCITY, &variable<VELOCITY>},
     {ACCELERATION, &variable<ACCELERATION>}};

const char *errorCodeToMessage(int error_code) {
  switch (error_code) {
  case control_msgs::FollowJointTrajectoryResult::SUCCESSFUL:
    return "SUCCESSFUL";
  case control_msgs::FollowJointTrajectoryResult::INVALID_GOAL:
    return "INVALID_GOAL";
  case control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS:
    return "INVALID_JOINTS";
  case control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP:
    return "OLD_HEADER_TIMESTAMP";
  case control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED:
    return "PATH_TOLERANCE_VIOLATED";
  case control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED:
    return "GOAL_TOLERANCE_VIOLATED";
  default:
    return "unknown error";
  }
}
} // namespace

void FollowJointGSplineControllerHandle::configure(
    XmlRpc::XmlRpcValue &config, const std::string &config_name,
    std::vector<control_msgs::JointTolerance> &tolerances) {
  if (moveit::core::isStruct(config)) // config should be either a struct of
                                      // position, velocity, acceleration
  {
    for (ToleranceVariables var : {POSITION, VELOCITY, ACCELERATION}) {
      if (!config.hasMember(VAR_NAME[var]))
        continue;
      XmlRpc::XmlRpcValue values = config[VAR_NAME[var]];
      if (moveit::core::isArray(values, joints_.size())) {
        size_t i = 0;
        for (const auto &joint_name : joints_)
          VAR_ACCESS[var](getTolerance(tolerances, joint_name)) =
              moveit::core::parseDouble(values[i++]);
      } else { // use common value for all joints
        double value = moveit::core::parseDouble(values);
        for (const auto &joint_name : joints_)
          VAR_ACCESS[var](getTolerance(tolerances, joint_name)) = value;
      }
    }
  } else if (moveit::core::isArray(
                 config)) // or an array of JointTolerance msgs
  {
    for (int i = 0; i < config.size(); ++i) // NOLINT(modernize-loop-convert)
    {
      control_msgs::JointTolerance &tol =
          getTolerance(tolerances, config[i]["name"]);
      for (ToleranceVariables var : {POSITION, VELOCITY, ACCELERATION}) {
        if (!config[i].hasMember(VAR_NAME[var]))
          continue;
        VAR_ACCESS[var](tol) =
            moveit::core::parseDouble(config[i][VAR_NAME[var]]);
      }
    }
  } else {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Invalid " << config_name);
  }
}

control_msgs::JointTolerance &FollowJointGSplineControllerHandle::getTolerance(
    std::vector<control_msgs::JointTolerance> &tolerances,
    const std::string &name) {
  auto it =
      std::lower_bound(tolerances.begin(), tolerances.end(), name,
                       [](const control_msgs::JointTolerance &lhs,
                          const std::string &rhs) { return lhs.name < rhs; });
  if (it == tolerances.cend() ||
      it->name != name) { // insert new entry if not yet available
    it = tolerances.insert(it, control_msgs::JointTolerance());
    it->name = name;
  }
  return *it;
}

void FollowJointGSplineControllerHandle::controllerDoneCallback(
    const actionlib::SimpleClientGoalState &state,
    const gsplines_msgs::FollowJointGSplineResultConstPtr &result) {
  // Output custom error message for FollowJointTrajectoryResult if necessary
  if (!result) {
    ROS_WARN_STREAM_NAMED(
        LOGNAME, "Controller '" << name_ << "' done, no result returned");
  } else if (result->error_code ==
             control_msgs::FollowJointTrajectoryResult::SUCCESSFUL) {
    ROS_INFO_STREAM_NAMED(LOGNAME,
                          "Controller '" << name_ << "' successfully finished");
  } else {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Controller '"
                                       << name_ << "' failed with error "
                                       << errorCodeToMessage(result->error_code)
                                       << ": " << result->error_string);
  }
  finishControllerExecution(state);
}

void FollowJointGSplineControllerHandle::controllerActiveCallback() {
  ROS_DEBUG_STREAM_NAMED(LOGNAME, name_ << " started execution");
}

void FollowJointGSplineControllerHandle::controllerFeedbackCallback(
    const gsplines_msgs::FollowJointGSplineFeedbackConstPtr & /* feedback */) {}

} // end namespace gsplines_moveit
