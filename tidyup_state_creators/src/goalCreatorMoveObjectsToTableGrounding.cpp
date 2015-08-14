#include "tidyup_state_creators/goalCreatorMoveObjectsToTableGrounding.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorMoveObjectsToTableGrounding, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{

    GoalCreatorMoveObjectsToTableGrounding::GoalCreatorMoveObjectsToTableGrounding()
    {
    }

    GoalCreatorMoveObjectsToTableGrounding::~GoalCreatorMoveObjectsToTableGrounding()
    {
    }

    void GoalCreatorMoveObjectsToTableGrounding::initialize(const std::deque<std::string> & arguments)
    {
    }

    bool GoalCreatorMoveObjectsToTableGrounding::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
    	goal.setForEachGoalStatement("table", "table-inspected", true);
    	//goal.setForEachGoalStatement("movable_object", "object-inspected", true);
    	goal.setForEachGoalStatement("arm", "hand-free", true); // derived predicate

    	//goal.setStringGoalStatement("(forall (?o - movable_object) (object-on ?o table2))");
        return true;
    }

};

