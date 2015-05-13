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



		// UGLY HACK IN STATECREATORFROMPLANNINGSCENE IN LINE 92
		// state.setBooleanPredicate("object-inspected", object.id, true);
		// BUT THERE THE UPDATE OF SYMBOLIC STATE WORKS

		ROS_WARN("ActionExecutorInspectObject::%s: Setting (%s, %s, true)", __func__,
				predicate_object_inspected_.c_str(), object.c_str());
		// update symbolic state
		currentState.setBooleanPredicate(predicate_object_inspected_, object, true);
		currentState.setBooleanPredicate("object-inspected", object, true);

		bool test = false;
		Predicate pred;
		pred.parameters.push_back(predicate_object_inspected_);
		if (!currentState.hasBooleanPredicate(pred, &test))
		{
			ROS_ERROR("ActionExecutorInspectObject::%s: Could not find predicate %s", __func__,
					predicate_object_inspected_.c_str());
			return false;
		}

		if (test)
			ROS_WARN("ActionExecutorInspectObject::%s: predicate was set successfully", __func__);
		else
		{
			ROS_ERROR("ActionExecutorInspectObject::%s: predicate was not set", __func__);
			return false;
		}

		return true;
	}

	void ActionExecutorInspectObject::cancelAction()
	{

	}

};

