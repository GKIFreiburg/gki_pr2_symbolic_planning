#include "tidyup_state_creators/goalCreatorTidyupInitializeGrounding.h"
#include "tidyup_utils/geometryPoses.h"
#include "tidyup_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>
#include <moveit_msgs/LoadMap.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorTidyupInitializeGrounding, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{
    GoalCreatorTidyupInitializeGrounding::GoalCreatorTidyupInitializeGrounding()
    {
    }

    GoalCreatorTidyupInitializeGrounding::~GoalCreatorTidyupInitializeGrounding()
    {
    }

    void GoalCreatorTidyupInitializeGrounding::initialize(const std::deque<std::string> & arguments)
    {
    }

    bool GoalCreatorTidyupInitializeGrounding::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        ros::NodeHandle nhPriv("~");
        // first add the type hierarchy
        currentState.addSuperType("pose", "pose");
        currentState.addSuperType("frameid", "frameid");
        currentState.addSuperType("location", "pose");
        currentState.addSuperType("table", "pose");
        currentState.addSuperType("movable_object", "pose");
        currentState.addSuperType("arm", "arm");
        currentState.addSuperType("arm_state", "arm_state");

        goal.addSuperType("pose", "pose");
        goal.addSuperType("frameid", "frameid");
        goal.addSuperType("location", "pose");
        goal.addSuperType("table", "pose");
        goal.addSuperType("movable_object", "pose");
        goal.addSuperType("arm", "arm");
        goal.addSuperType("arm_state", "arm_state");

        currentState.printSuperTypes();

        currentState.setObjectFluent("arm-state", "right_arm", "arm_unknown");
        currentState.setObjectFluent("arm-state", "left_arm", "arm_unknown");

        return true;
    }

};

