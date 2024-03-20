

#include <gsplines_moveit/GSplinesControllerHandler.hpp>
#include <gsplines_msgs/GetBasis.h>
#include <map>
#include <moveit/utils/xmlrpc_casts.h>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <moveit_simple_controller_manager/gripper_controller_handle.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

using namespace moveit::core;

const std::string LOGNAME("GSplinesControllerManager");

namespace gsplines_moveit {
class GSplinesControllerManager
    : public moveit_controller_manager::MoveItControllerManager {
public:
  GSplinesControllerManager() : node_handle_("~") {

    /// 0. try to get controller list
    if (!node_handle_.hasParam("controller_list")) {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "No controller_list specified.");
      return;
    }

    ///    0.1 load the controller_list
    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (!isArray(controller_list)) {
      ROS_ERROR_NAMED(
          LOGNAME, "Parameter controller_list should be specified as an array");
      return;
    }

    /* actually create each controller */
    for (int i = 0; i < controller_list.size();
         ++i) // NOLINT(modernize-loop-convert)
    {
      if (!isStruct(controller_list[i],
                    {"name", "joints", "action_ns", "type"})) {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "name, joints, action_ns, and type "
                                        "must be specifed for each controller");
        continue;
      }

      try {
        const std::string name = std::string(controller_list[i]["name"]);
        const std::string action_ns =
            std::string(controller_list[i]["action_ns"]);
        const std::string type = std::string(controller_list[i]["type"]);

        if (!isArray(controller_list[i]["joints"])) {
          ROS_ERROR_STREAM_NAMED(
              LOGNAME, "The list of joints for controller "
                           << name << " is not specified as an array");
          continue;
        }

        moveit_simple_controller_manager::ActionBasedControllerHandleBasePtr
            new_handle;

        if (type == "GripperCommand") {
          const double max_effort =
              controller_list[i].hasMember("max_effort")
                  ? double(controller_list[i]["max_effort"])
                  : 0.0;

          new_handle = std::make_shared<
              moveit_simple_controller_manager::GripperControllerHandle>(
              name, action_ns, max_effort);
          if (static_cast<
                  moveit_simple_controller_manager::GripperControllerHandle *>(
                  new_handle.get())
                  ->isConnected()) {
            if (controller_list[i].hasMember("parallel")) {
              if (controller_list[i]["joints"].size() != 2) {
                ROS_ERROR_STREAM_NAMED(
                    LOGNAME, "Parallel Gripper requires exactly two joints");
                continue;
              }
              static_cast<
                  moveit_simple_controller_manager::GripperControllerHandle *>(
                  new_handle.get())
                  ->setParallelJawGripper(controller_list[i]["joints"][0],
                                          controller_list[i]["joints"][1]);
            } else {
              if (controller_list[i].hasMember("command_joint"))
                static_cast<moveit_simple_controller_manager::
                                GripperControllerHandle *>(new_handle.get())
                    ->setCommandJoint(controller_list[i]["command_joint"]);
              else
                static_cast<moveit_simple_controller_manager::
                                GripperControllerHandle *>(new_handle.get())
                    ->setCommandJoint(controller_list[i]["joints"][0]);
            }

            if (controller_list[i].hasMember("allow_failure")) {
              static_cast<
                  moveit_simple_controller_manager::GripperControllerHandle *>(
                  new_handle.get())
                  ->allowFailure(true);
            }

            ROS_INFO_STREAM_NAMED(
                LOGNAME, "Added GripperCommand controller for " << name);
            controllers_[name] = new_handle;
          }
        } else if (type == "FollowJointGSpline") {
          auto h = new FollowJointGSplineControllerHandle(name, action_ns);
          new_handle.reset(h);
          if (h->isConnected()) {
            ROS_INFO_STREAM_NAMED(
                LOGNAME, "Added FollowJointGSpline controller for " << name);
            controllers_[name] = new_handle;
          }
        } else {
          ROS_ERROR_STREAM_NAMED(LOGNAME,
                                 "Unknown controller type: " << type.c_str());
          continue;
        }
        if (!controllers_[name]) {
          controllers_.erase(name);
          continue;
        }

        moveit_controller_manager::MoveItControllerManager::ControllerState
            state;
        state.default_ = controller_list[i].hasMember("default")
                             ? (bool)controller_list[i]["default"]
                             : false;
        state.active_ = true;

        controller_states_[name] = state;

        /* add list of joints, used by controller manager and MoveIt */
        for (int j = 0; j < controller_list[i]["joints"].size(); ++j) {
          new_handle->addJoint(std::string(controller_list[i]["joints"][j]));
        }

        new_handle->configure(controller_list[i]);
      } catch (...) {
        ROS_ERROR_STREAM_NAMED(
            LOGNAME,
            "Caught unknown exception while parsing controller information");
      }
    }

    get_basis_ = this->node_handle_.serviceClient<gsplines_msgs::GetBasis>(
        "gsplines_moveit/get_basis");
  }

  ros::ServiceClient get_basis_;

  ~GSplinesControllerManager() override = default;

  /*
   * Get a controller, by controller name (which was specified in the
   * controllers.yaml
   */
  moveit_controller_manager::MoveItControllerHandlePtr
  getControllerHandle(const std::string &name) override {
    const auto it = controllers_.find(name);
    if (it != controllers_.end()) {
      return it->second;
    } else {
      ROS_FATAL_STREAM_NAMED(LOGNAME, "No such controller: " << name);
    }
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  void getControllersList(std::vector<std::string> &names) override {
    for (const auto &it : controllers_) {
      names.push_back(it.first);
    }
    ROS_INFO_STREAM_NAMED(LOGNAME, "Returned " << names.size()
                                               << " controllers in list");
  }

  /*
   * This plugin assumes that all controllers are already active -- and if they
   * are not, well, it has no way to deal with it anyways!
   */
  void getActiveControllers(std::vector<std::string> &names) override {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active
   * controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string> &names) {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  void getControllerJoints(const std::string &name,
                           std::vector<std::string> &joints) override {
    std::map<std::string,
             moveit_simple_controller_manager::
                 ActionBasedControllerHandleBasePtr>::const_iterator it =
        controllers_.find(name);
    if (it != controllers_.end()) {
      it->second->getJoints(joints);
    } else {
      ROS_WARN_NAMED(LOGNAME,
                     "The joints for controller '%s' are not known. Perhaps "
                     "the controller configuration is "
                     "not loaded on the param server?",
                     name.c_str());
      joints.clear();
    }
  }

  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string &name) override {
    return controller_states_[name];
  }

  /* Cannot switch our controllers */
  bool switchControllers(
      const std::vector<std::string> & /* activate */,
      const std::vector<std::string> & /* deactivate */) override {
    return false;
  }

protected:
  ros::NodeHandle node_handle_;
  std::map<std::string,
           moveit_simple_controller_manager::ActionBasedControllerHandleBasePtr>
      controllers_;
  std::map<std::string,
           moveit_controller_manager::MoveItControllerManager::ControllerState>
      controller_states_;
};

} // namespace gsplines_moveit

PLUGINLIB_EXPORT_CLASS(gsplines_moveit::GSplinesControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
