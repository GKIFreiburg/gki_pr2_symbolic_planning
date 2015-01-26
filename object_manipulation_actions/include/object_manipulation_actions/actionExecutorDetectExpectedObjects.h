#ifndef ACTION_EXECUTOR_DETECT_GRASPABLE_OBJECTS_H
#define ACTION_EXECUTOR_DETECT_GRASPABLE_OBJECTS_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <actionlib/client/simple_action_client.h>
#include <tidyup_msgs/DetectExpectedGraspableObjects.h>

namespace object_manipulation_actions
{

class ActionExecutorDetectExpectedObjects: public ActionExecutorService<tidyup_msgs::DetectExpectedGraspableObjects>
{
public:
    virtual void initialize(const std::deque<std::string> & arguments);

    virtual bool fillGoal(tidyup_msgs::DetectExpectedGraspableObjects::Request & goal, const DurativeAction & a, const SymbolicState & current);

    virtual void updateState(bool & success, tidyup_msgs::DetectExpectedGraspableObjects::Response & response, const DurativeAction & a, SymbolicState & current);

private:
    ros::ServiceClient serviceClientGraspability;
    bool requestGraspability;
    string tidyLocationName;

    std::string findStaticObjectForLocation(const std::string& location, const SymbolicState & current) const;
};

}
;

#endif

