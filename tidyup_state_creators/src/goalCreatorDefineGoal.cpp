#include "tidyup_state_creators/goalCreatorDefineGoal.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorDefineGoal, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{

    GoalCreatorDefineGoal::GoalCreatorDefineGoal()
    {
    }

    GoalCreatorDefineGoal::~GoalCreatorDefineGoal()
    {
    }

    void GoalCreatorDefineGoal::initialize(const std::deque<std::string> & arguments)
    {
    }

    bool GoalCreatorDefineGoal::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
    	goal.setForEachGoalStatement("table", "table-inspected", true);
    	goal.setForEachGoalStatement("movable_object", "object-inspected", true);
    	goal.setForEachGoalStatement("arm", "hand-free", true); // derived predicate
        return true;
    }

};

