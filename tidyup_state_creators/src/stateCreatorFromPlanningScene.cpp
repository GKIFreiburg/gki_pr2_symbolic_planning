#include "tidyup_utils/stringutil.h"
#include <tidyup_state_creators/stateCreatorFromPlanningScene.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorFromPlanningScene, continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{
    StateCreatorFromPlanningScene::StateCreatorFromPlanningScene()
    {
    }

    StateCreatorFromPlanningScene::~StateCreatorFromPlanningScene()
    {
    }

    void StateCreatorFromPlanningScene::initialize(const std::deque<std::string> & arguments)
    {
    }

    bool StateCreatorFromPlanningScene::fillState(SymbolicState & state)
    {
        return true;
    }

};

