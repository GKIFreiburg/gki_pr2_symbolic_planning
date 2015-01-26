#include "tidyup_state_creators/goalCreatorMoveObjectsToTable.h"
#include "tidyup_utils/geometryPoses.h"
#include "tidyup_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>

PLUGINLIB_DECLARE_CLASS(tidyup_state_creators, goal_creator_move_objects_to_table,
        tidyup_state_creators::GoalCreatorMoveObjectsObjectsToTable, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{

    GoalCreatorMoveObjectsObjectsToTable::GoalCreatorMoveObjectsObjectsToTable()
    {
    }

    GoalCreatorMoveObjectsObjectsToTable::~GoalCreatorMoveObjectsObjectsToTable()
    {
    }

    bool GoalCreatorMoveObjectsObjectsToTable::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        goal.setForEachGoalStatement("manipulation_location", "searched", true);
        goal.setForEachGoalStatement("movable_object", "tidy", true);
        goal.setForEachGoalStatement("arm", "hand-free", true);
        goal.setForEachGoalStatement("wipe_point", "wiped", true);

        return true;
    }

};

