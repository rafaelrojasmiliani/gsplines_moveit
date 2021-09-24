#ifndef MINIMUMSOBOLEVSEMINORMADAPTER_H
#define MINIMUMSOBOLEVSEMINORMADAPTER_H
#include <class_loader/class_loader.hpp>
#include <moveit/planning_request_adapter/planning_request_adapter.h>

class MinimumSobolevSeminormAdapter
    : public planning_request_adapter::PlanningRequestAdapter {
private:
public:
  MinimumSobolevSeminormAdapter();
  virtual ~MinimumSobolevSeminormAdapter();
};

#endif /* MINIMUMSOBOLEVSEMINORMADAPTER_H */
