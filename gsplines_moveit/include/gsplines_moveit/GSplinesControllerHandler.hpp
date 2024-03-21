#pragma once

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <gsplines_msgs/FollowJointGSplineAction.h>
#include <gsplines_msgs/GetBasis.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace gsplines_moveit {
/*
 * Controls anything controlled by gsplines_mgs/FollowJointGSplineAction.
 */
class FollowJointGSplineControllerHandle
    : public moveit_simple_controller_manager::ActionBasedControllerHandle<
          gsplines_msgs::FollowJointGSplineAction> {
public:
  ros::NodeHandle node_handle_{"~"};
  ros::ServiceClient get_basis_ =
      node_handle_.serviceClient<gsplines_msgs::GetBasis>(
          "gsplines_moveit/get_basis");

  FollowJointGSplineControllerHandle(const std::string &name,
                                     const std::string &action_ns)
      : moveit_simple_controller_manager::ActionBasedControllerHandle<
            gsplines_msgs::FollowJointGSplineAction>(name, action_ns) {

    ROS_WARN_STREAM("gspline andher!");
  }

  bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory) override;

  void configure(XmlRpc::XmlRpcValue &config) override;

protected:
  void configure(XmlRpc::XmlRpcValue &config, const std::string &config_name,
                 std::vector<control_msgs::JointTolerance> &tolerances);
  static control_msgs::JointTolerance &
  getTolerance(std::vector<control_msgs::JointTolerance> &tolerances,
               const std::string &name);

  void controllerDoneCallback(
      const actionlib::SimpleClientGoalState &state,
      const gsplines_msgs::FollowJointGSplineResultConstPtr &result);

  void controllerActiveCallback();

  void controllerFeedbackCallback(
      const gsplines_msgs::FollowJointGSplineFeedbackConstPtr &feedback);

  gsplines_msgs::FollowJointGSplineGoal goal_template_;
};

} // end namespace gsplines_moveit
