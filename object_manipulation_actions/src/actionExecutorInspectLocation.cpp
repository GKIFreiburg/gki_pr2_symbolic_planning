#include "object_manipulation_actions/actionExecutorInspectLocation.h"
#include <pluginlib/class_list_macros.h>
#include <ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h>
#include <symbolic_planning_utils/extractPose.h>
//#include <moveit/move_group_interface/move_group.h>

#include <set>
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorInspectLocation, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
ActionExecutorInspectLocation::ActionExecutorInspectLocation() :
		actionLiftTorso_("torso_controller/position_joint_action", true),
		actionPointHead_("head_traj_controller/point_head_action", true),
		actionOrkToPs_("ork_to_planning_scene", true)
{
	// TODO: read from param server
	actionTimeOut_ = ros::Duration(30.0);
	vDistHeadToTable_ = 0.5; // m

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
	ROS_ASSERT(arguments.size() >= 3);
	action_name_ = arguments[0];
	action_topic_ = arguments[1];
	for (int i = 2; i < arguments.size(); i++)
	{
		predicate_names_.push_back(arguments[i]);
	}

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
		ROS_ERROR("ActionExecutorInspectLocation::%s: Could not extract pose of table:%s from symbolic state", __func__, table.c_str());
		return false;
	}

	if (!executeLiftTorso(tablePose))
		return false;

	if (!executePointHead(tablePose))
		return false;

	if (!executeUpdatePlanningSceneFromORK(currentState, table, mani_loc))
		return false;


	return true;
}

void ActionExecutorInspectLocation::cancelAction()
{

}

void ActionExecutorInspectLocation::feedbackLiftTorso(const control_msgs::SingleJointPositionFeedback& feedback)
{
	ROS_DEBUG_STREAM("Torso Joint Position: " << feedback.position << " , Error: " << feedback.error);
//	if (feedback.error < 0.01)
//		actionLiftTorso_.cancelGoal();
}

bool ActionExecutorInspectLocation::executeLiftTorso(const geometry_msgs::PoseStamped tablePose)
{
	// compute difference in height between head_mount_link and table
    // z coord from tf pose in /base_footprint frame
    geometry_msgs::PoseStamped pose_transformed;
    try {
        tf_.waitForTransform("/head_mount_link", tablePose.header.frame_id, tablePose.header.stamp,
                ros::Duration(0.5));
        tf_.transformPose("/head_mount_link", tablePose, pose_transformed);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    if (!abs(pose_transformed.pose.position.z) < vDistHeadToTable_)
    	// head_mount_link is more than vDistHeadToTable above table, no need to lift torso
    	return true;

	ROS_INFO("ActionExecutorInspectLocation::%s: Sending ListTorso request.", __func__);
	control_msgs::SingleJointPositionGoal liftTorsoGoal;
	liftTorsoGoal.position = vDistHeadToTable_ - abs(pose_transformed.pose.position.z);

	actionLiftTorso_.sendGoal(liftTorsoGoal);

	bool finished_before_timeout = actionLiftTorso_.waitForResult(actionTimeOut_);
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionLiftTorso_.getState();
		ROS_DEBUG("ActionExecutorInspectLocation::%s: Lift Torso Action finished: %s", __func__, state.toString().c_str());
		if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("ActionExecutorInspectLocation::%s: Lift Torso Action failed.", __func__);
			return false;
		}
	}
	return true;
}

bool ActionExecutorInspectLocation::executePointHead(const geometry_msgs::PoseStamped tablePose)
{
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

	bool finished_before_timeout = actionPointHead_.waitForResult(actionTimeOut_);
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionPointHead_.getState();
		ROS_DEBUG("ActionExecutorInspectLocation::%s: Point Head Action finished: %s", __func__, state.toString().c_str());
		if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("ActionExecutorInspectLocation::%s: Point Head Action failed.", __func__);
			return false;
		}
	}
	return true;
}

bool ActionExecutorInspectLocation::executeUpdatePlanningSceneFromORK(SymbolicState& currentState,
		const std::string tableName, const std::string manipulationLocation)
{
	// execute action ork-to-planning-scene
	ROS_INFO("ActionExecutorInspectLocation::%s: Sending UpdatePlanningSceneFromORK request.", __func__);
	ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoal updatePSGoal;
	updatePSGoal.verify_planning_scene_update = verify_planning_scene_update_;
	updatePSGoal.add_tables = add_tables_;
	updatePSGoal.table_prefix = tableName;

	const multimap<string, string> objects = currentState.getTypedObjects();
	std::pair<multimap<string, string>::const_iterator, multimap<string, string>::const_iterator> iterators = objects.equal_range("movable_object");
	for (multimap<string, string>::const_iterator it = iterators.first; it != iterators.second; it++)
	{
		Predicate on;
		on.name = "object-on";
		on.parameters.push_back(it->second);
		on.parameters.push_back(tableName);
		bool value = false;
		currentState.hasBooleanPredicate(on, &value);
		if (value)
		{
			updatePSGoal.expected_objects.push_back(it->second);
		}
	}
	actionOrkToPs_.sendGoal(updatePSGoal);
	bool finished_before_timeout = actionOrkToPs_.waitForResult(actionTimeOut_);
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionOrkToPs_.getState();
		ROS_DEBUG("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action finished: %s", __func__, state.toString().c_str());
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Inspect location succeeded.");
			for_each(const string& predicate, predicate_names_)
			{
				currentState.setBooleanPredicate(predicate, manipulationLocation, true);
			}
		}
		else
		{
			ROS_ERROR("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action failed.", __func__);
			return false;
		}
	}
	return true;
}

}; // namespace
