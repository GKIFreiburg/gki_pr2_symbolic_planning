#include "object_manipulation_actions/actionExecutorInspectLocation.h"
#include <pluginlib/class_list_macros.h>
#include <ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <set>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorInspectLocation, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
	void ActionExecutorInspectLocation::initialize(const std::deque<std::string> & arguments)
	{

	}

	bool ActionExecutorInspectLocation::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
	    return a.name == action_name_;
	}

	bool ActionExecutorInspectLocation::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{

	    return false;
	}

	void ActionExecutorInspectLocation::cancelAction()
	{

	}

};

