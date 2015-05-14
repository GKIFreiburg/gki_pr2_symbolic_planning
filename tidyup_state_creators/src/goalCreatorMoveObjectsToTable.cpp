#include "tidyup_state_creators/goalCreatorMoveObjectsToTable.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorMoveObjectsToTable, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{

    GoalCreatorMoveObjectsToTable::GoalCreatorMoveObjectsToTable()
    {
    }

    GoalCreatorMoveObjectsToTable::~GoalCreatorMoveObjectsToTable()
    {
    }

    void GoalCreatorMoveObjectsToTable::initialize(const std::deque<std::string> & arguments)
    {
    }

    bool GoalCreatorMoveObjectsToTable::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
    	goal.setForEachGoalStatement("manipulation_location", "location-inspected", true);
    	goal.setForEachGoalStatement("movable_object", "object-inspected", true);
    	goal.setForEachGoalStatement("arm", "hand-free", true); // derived predicate

    	//goal.setStringGoalStatement("(forall (?o - movable_object) (object-on ?o table2))");
        return true;
    }

};

