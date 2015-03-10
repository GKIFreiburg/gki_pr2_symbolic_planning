#include "object_manipulation_actions/actionExecutorInspectLocation.h"
#include <pluginlib/class_list_macros.h>
#include <set>

//PLUGINLIB_DECLARE_CLASS(object_manipulation_actions, action_executor_detect_objects,
//		object_manipulation_actions::ActionExecutorInspectLocation,
//		continual_planning_executive::ActionExecutorInterface)
PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorInspectLocation, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
	void ActionExecutorInspectLocation::initialize(const std::deque<std::string> & arguments)
	{
		ActionExecutorActionlib<
				ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkAction,
				ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoal,
				ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResult>::initialize(arguments);

		ROS_ASSERT(arguments.size() == 3);
        action_name_    = arguments[0];
        action_topic_   = arguments[1];
        predicate_name_ = arguments[2];

        add_tables_ = true;
        verify_planning_scene_update_ = false;
		return;
	}

	bool ActionExecutorInspectLocation::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
	    return a.name == action_name_;
	}

	bool ActionExecutorInspectLocation::fillGoal(ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoal & goal,
			const DurativeAction & a, const SymbolicState & current)
	{
		ROS_ASSERT(a.parameters.size() == 2);
		std::string mani_loc = a.parameters[0];
		std::string table = a.parameters[1];

		goal.verify_planning_scene_update = verify_planning_scene_update_;
		//goal.expected_objects.insert(goal.expected_objects.end(), expected_objects_.begin(), expected_objects_.end());
		goal.add_tables = add_tables_;
		goal.table_prefix = table;

		const multimap<string, string> objects = current.getTypedObjects();
		std::pair<multimap<string, string>::const_iterator, multimap<string, string>::const_iterator> iterators =
		        objects.equal_range("movable_object");
		for (multimap<string, string>::const_iterator it = iterators.first; it != iterators.second; it++)
		{
		    goal.expected_objects.push_back(it->second);
		}

		return true;
	}

	void ActionExecutorInspectLocation::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
    		const ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResult & result,
            const DurativeAction & a, SymbolicState & current)
	{
		ROS_ASSERT(a.parameters.size() == 2);
		std::string mani_loc = a.parameters[0];
		std::string table = a.parameters[1];

        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Inspect location succeeded.");
            // correct?
            current.setBooleanPredicate(predicate_name_, mani_loc, true);
        }

		return;
	}

};

