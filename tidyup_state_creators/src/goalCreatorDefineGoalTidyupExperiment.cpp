#include "tidyup_state_creators/goalCreatorDefineGoalTidyupExperiment.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorDefineGoalTidyupExperiment, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{

    GoalCreatorDefineGoalTidyupExperiment::GoalCreatorDefineGoalTidyupExperiment()
    {
    }

    GoalCreatorDefineGoalTidyupExperiment::~GoalCreatorDefineGoalTidyupExperiment()
    {
    }

    void GoalCreatorDefineGoalTidyupExperiment::initialize(const std::deque<std::string> & arguments)
    {
    }

    bool GoalCreatorDefineGoalTidyupExperiment::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
    	std::stringstream ss;
    	ss << "(and" << std::endl;
    	ss << "      (forall (?o - cube) (object-on ?o table1))" << std::endl;
    	ss << "      (forall (?o - can) (object-on ?o table2))" << std::endl;
    	ss << "      (forall (?o - arm) (hand-free ?o))" << std::endl;
    	ss << "      (forall (?o - table) (table-inspected ?o))" << std::endl;
    	ss << "      (arms-drive-pose)" << std::endl;
    	ss << "    )";

    	goal.setStringGoalStatement(ss.str());

        return true;
    }

};

