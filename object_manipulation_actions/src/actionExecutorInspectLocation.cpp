#include "object_manipulation_actions/actionExecutorInspectLocation.h"
#include <pluginlib/class_list_macros.h>
#include <ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h>
#include <symbolic_planning_utils/extractPose.h>
//#include <moveit/move_group_interface/move_group.h>
#include <set>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorInspectLocation, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
	ActionExecutorInspectLocation::ActionExecutorInspectLocation()	:
			actionPointHead_("head_traj_controller/point_head_action", true),
			actionOrkToPs_("ork_to_planning_scene", true)
	{
	    ROS_DEBUG("ActionExecutorInspectLocation::%s: Waiting for head_traj_controller/point_head_action"
	    		"action server to start.", __func__);
	    actionPointHead_.waitForServer();

	    ROS_DEBUG("ActionExecutorInspectLocation::%s: Action client ready", __func__);
	}

	ActionExecutorInspectLocation::~ActionExecutorInspectLocation()
	{
	}

	void ActionExecutorInspectLocation::initialize(const std::deque<std::string> & arguments)
	{
		ROS_ASSERT(arguments.size() == 3);
        action_name_    = arguments[0];
        action_topic_   = arguments[1];
        predicate_name_ = arguments[2];

        add_tables_ = false;
        verify_planning_scene_update_ = true;
	}

	bool ActionExecutorInspectLocation::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
	    return a.name == action_name_;
	}

	bool ActionExecutorInspectLocation::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
		// point head to tablePose
		ROS_ASSERT(a.parameters.size() == 2);
		std::string mani_loc = a.parameters[0];
		std::string table = a.parameters[1];

		// first get tablePose
		geometry_msgs::PoseStamped tablePose;
		if (!symbolic_planning_utils::extractPoseStampedFromSymbolicState(currentState, table, tablePose))
		{
			ROS_ERROR("ActionExecutorInspectLocation::%s: Could not extract pose of table:%s from symbolic state",
					__func__, table.c_str() );
			return false;
		}

	    ROS_INFO("ActionExecutorInspectLocation::%s: Sending PointHead request.", __func__);
	    control_msgs::PointHeadGoal pointHeadGoal;
	    geometry_msgs::PointStamped target;
	    // Set the target pose to the pose of the table
	    target.point = tablePose.pose.position;
	    target.header = tablePose.header;
	    pointHeadGoal.target = target;
	    // Using x-axis of frame "head_mount_link" for pointing
	    pointHeadGoal.pointing_axis.x = 1;
	    pointHeadGoal.pointing_axis.y = 0;
	    pointHeadGoal.pointing_axis.z = 0;
	    pointHeadGoal.pointing_frame = "head_mount_link";

	    actionPointHead_.sendGoal(pointHeadGoal);

	    bool finished_before_timeout = actionPointHead_.waitForResult(ros::Duration(30.0));
	    if(finished_before_timeout) {
	    	actionlib::SimpleClientGoalState state = actionPointHead_.getState();
	    	ROS_DEBUG("ActionExecutorInspectLocation::%s: Point Head Action finished: %s",
	    			__func__, state.toString().c_str());
	        if(state != actionlib::SimpleClientGoalState::SUCCEEDED) {
	            ROS_ERROR("ActionExecutorInspectLocation::%s: Point Head Action failed.", __func__);
	            return false;
	        }
	    }

	    // execute action ork-to-planning-scene
	    ROS_INFO("ActionExecutorInspectLocation::%s: Sending UpdatePlanningSceneFromORK request.", __func__);
	    ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoal updatePSGoal;
	    updatePSGoal.verify_planning_scene_update = verify_planning_scene_update_;
	    updatePSGoal.add_tables = add_tables_;
	    updatePSGoal.table_prefix = table;

		const multimap<string, string> objects = currentState.getTypedObjects();
		std::pair<multimap<string, string>::const_iterator, multimap<string, string>::const_iterator> iterators =
		        objects.equal_range("movable_object");
		for (multimap<string, string>::const_iterator it = iterators.first; it != iterators.second; it++)
		{
		    Predicate on;
		    on.name = "object-on";
		    on.parameters.push_back(it->second);
		    on.parameters.push_back(table);
		    bool value = false;
		    currentState.hasBooleanPredicate(on, &value);
		    if (value)
		    {
		    	updatePSGoal.expected_objects.push_back(it->second);
		    }
		}
		actionOrkToPs_.sendGoal(updatePSGoal);
		finished_before_timeout = actionOrkToPs_.waitForResult(ros::Duration(30.0));
	    if (finished_before_timeout) {
	    	actionlib::SimpleClientGoalState state = actionOrkToPs_.getState();
	    	ROS_DEBUG("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action finished: %s",
	    			__func__, state.toString().c_str());
	        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
	        	ROS_INFO("Inspect location succeeded.");
	        	currentState.setBooleanPredicate(predicate_name_, mani_loc, true);
	        }
	        else {
	            ROS_ERROR("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action failed.", __func__);
	            return false;
	        }
	    }

		return true;
	}

	void ActionExecutorInspectLocation::cancelAction()
	{

	}

};

