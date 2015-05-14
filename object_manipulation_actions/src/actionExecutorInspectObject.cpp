#include "object_manipulation_actions/actionExecutorInspectObject.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorInspectObject, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
	ActionExecutorInspectObject::ActionExecutorInspectObject()
	{

	}

	ActionExecutorInspectObject::~ActionExecutorInspectObject()
	{

	}

	void ActionExecutorInspectObject::initialize(const std::deque<std::string> & arguments)
	{
		ROS_ASSERT(arguments.size() == 2);
		action_name_ 			 	= arguments[0];		// inspect-object
		predicate_object_inspected_ = arguments[1];		// object-inspected
	}

	bool ActionExecutorInspectObject::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
		return a.name == action_name_;
	}

	bool ActionExecutorInspectObject::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
		ROS_ASSERT(a.parameters.size() == 2);
		std::string object   = a.parameters[0];
		std::string arm 	 = a.parameters[1];

		// TODO: do useful stuff

		// Set predicate object-inspected
		currentState.setBooleanPredicate(predicate_object_inspected_, object, true);

		return true;
	}

	void ActionExecutorInspectObject::cancelAction()
	{

	}

};

