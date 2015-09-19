#include "tidyup_state_creators/goalCreatorDefineGoalGrounding.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorDefineGoalGrounding, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{

    GoalCreatorDefineGoalGrounding::GoalCreatorDefineGoalGrounding()
    {
    }

    GoalCreatorDefineGoalGrounding::~GoalCreatorDefineGoalGrounding()
    {
    }

    void GoalCreatorDefineGoalGrounding::initialize(const std::deque<std::string> & arguments)
    {
    }

    bool GoalCreatorDefineGoalGrounding::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
    	goal.setForEachGoalStatement("table", "table-inspected", true);
    	goal.setForEachGoalStatement("movable_object", "object-inspected", true);
    	goal.setForEachGoalStatement("arm", "hand-free", true); // derived predicate
        return true;
    }

};

