#include "tidyup_state_creators/goalCreatorResetWorld.h"
#include "tidyup_utils/geometryPoses.h"
#include "tidyup_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>

//PLUGINLIB_DECLARE_CLASS(tidyup_state_creators, goal_creator_reset_world,
//        tidyup_state_creators::GoalCreatorResetWorld, continual_planning_executive::GoalCreator)
PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorResetWorld, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{
	bool GoalCreatorResetWorld::worldResetPerformed = false;

    GoalCreatorResetWorld::GoalCreatorResetWorld()
    {
    }

    GoalCreatorResetWorld::~GoalCreatorResetWorld()
    {
    }

    void GoalCreatorResetWorld::initialize(const std::deque<std::string> & arguments)
    {

    }

    bool GoalCreatorResetWorld::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        ros::NodeHandle nhPriv("~");
        if (! worldResetPerformed)
        {
            worldResetPerformed = true;
            std_srvs::Empty msg;
            ros::service::call("/tidyup/reset_world_interface", msg);
        }
        return true;
    }

};

