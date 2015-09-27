#include "object_manipulation_actions/actionExecutorInspectLocation.h"
#include <pluginlib/class_list_macros.h>
#include <ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h>
#include <control_msgs/SingleJointPositionActionFeedback.h>
#include <control_msgs/SingleJointPositionFeedback.h>
#include <tidyup_utils/stringutil.h>
#include <symbolic_planning_utils/extractPose.h>
#include <symbolic_planning_utils/planning_scene_monitor.h>
#include <symbolic_planning_utils/planning_scene_service.h>

#include <angles/angles.h>
#include <set>
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorInspectLocation, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{

void ActionExecutorInspectLocation::initialize(const std::deque<std::string> & arguments)
{
	ROS_ASSERT(arguments.size() == 6);
	action_name_ 						 = arguments[0];
	action_topic_ork_ps_     			 = arguments[1];
	action_topic_point_head_ 			 = arguments[2];
	pointing_frame_						 = arguments[3];
	predicate_location_inspected_  		 = arguments[4];
	predicate_table_inspected_recently_  = arguments[5];

	actionTimeOut_ = ros::Duration(30.0);
	joint_name_head_yaw_ = "head_pan_joint";
	head_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getHeadGroup();

	actionOrkToPs_ = new actionlib::SimpleActionClient<ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkAction>(action_topic_ork_ps_, true);
	ROS_INFO("ActionExecutorInspectLocation::%s: Waiting for ork_to_planning_scene "
			"action server to start.", __func__);
	actionOrkToPs_->waitForServer();

	actionPointHead_ = new actionlib::SimpleActionClient<control_msgs::PointHeadAction>(action_topic_point_head_, true);
	ROS_INFO("ActionExecutorInspectLocation::%s: Waiting for head_traj_controller/point_head_action "
			"action server to start.", __func__);
	actionPointHead_->waitForServer();

	ROS_INFO("ActionExecutorInspectLocation::%s: Action clients are ready", __func__);

    ros::NodeHandle nhPriv("~");
    // Namespace is "/continual_planning_executive"(/degrees)
    nhPriv.param("degrees", degrees_, 30);

	// better use service calls rather than planningSceneMontior,
	// which needs the initialize the robot description - takes time)
	// psi_.reset(new symbolic_planning_utils::PlanningSceneMonitor());
	psi_.reset(new symbolic_planning_utils::PlanningSceneService());
}

bool ActionExecutorInspectLocation::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
{
	return a.name == action_name_;
}

bool ActionExecutorInspectLocation::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
{
	ROS_ASSERT(a.parameters.size() == 2);
	std::string mani_loc = a.parameters[0];
	std::string table 	 = a.parameters[1];

	// get tablePose
	geometry_msgs::PoseStamped tablePose;
	if (!symbolic_planning_utils::extractPoseStampedFromSymbolicState(currentState, table, tablePose))
	{
		ROS_ERROR("ActionExecutorInspectLocation::%s: Could not extract pose of table:%s from symbolic state", __func__, table.c_str());
		return false;
	}

	executePointHead(tablePose);
	if (!executePointHead(tablePose))
		return false;

	// after head is pointed to table, execute visual detection
	bool verify_planning_scene_update = true;
	bool add_tables = true;
	bool merge_tables = false;
	std::string table_prefix = table;
	std::vector<std::string> expected_objects;

	// fill expected_objects
	const multimap<string, string> objects = currentState.getTypedObjects();
	std::pair<multimap<string, string>::const_iterator, multimap<string, string>::const_iterator> iterators = objects.equal_range("movable_object");
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
//			expected_objects.push_back(it->second);
			expected_objects_.insert(it->second);
		}
	}

	// sleep is needed before each object detection, so that the camera has chance to deliver good images
	ros::Duration(1.0).sleep();
	expected_objects.assign(expected_objects_.begin(), expected_objects_.end());
	if (!executeUpdatePlanningSceneFromORK(verify_planning_scene_update, expected_objects,
			add_tables, table_prefix, merge_tables))
		return false;

	// turn head by given degrees - return value is ignored, since action is executed but reports error
	if (!executeTurnHead(degrees_))
		return false;

	// execute visual detection to merge table
	merge_tables = true;
	expected_objects.assign(expected_objects_.begin(), expected_objects_.end());
	ros::Duration(1.0).sleep();
	if (!executeUpdatePlanningSceneFromORK(verify_planning_scene_update, expected_objects,
			add_tables, table_prefix, merge_tables))
		return false;

	// turn head by given degrees, this time in opposite direction
	if (!executeTurnHead(degrees_ * -1))
		return false;

	// execute visual detection to merge table
	merge_tables = true;
	expected_objects.assign(expected_objects_.begin(), expected_objects_.end());
	ros::Duration(1.0).sleep();
	if (!executeUpdatePlanningSceneFromORK(verify_planning_scene_update, expected_objects,
			add_tables, table_prefix, merge_tables))
		return false;

	// After operation is done, point head again to table
	if (!executePointHead(tablePose))
		return false;

	// after detection is completed, set predicates: location-inspected and location_inspected-recently
	currentState.setBooleanPredicate(predicate_location_inspected_, mani_loc, true);
	currentState.setBooleanPredicate(predicate_table_inspected_recently_, table, true);

	// Cut off _number from table detection and readd table to PS
	renameTableCollisionObject(table);

	return true;
}

void ActionExecutorInspectLocation::cancelAction()
{
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
	pointHeadGoal.pointing_frame = pointing_frame_;

	actionPointHead_->sendGoal(pointHeadGoal);

	bool finished_before_timeout = actionPointHead_->waitForResult(actionTimeOut_);
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionPointHead_->getState();
		ROS_DEBUG("ActionExecutorInspectLocation::%s: Point Head Action finished: %s", __func__, state.toString().c_str());
		if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("ActionExecutorInspectLocation::%s: Point Head Action failed.", __func__);
			//return false;
		}
	}

	// store the joint value for the pointed head position, needed by turning head
	std::vector<double> current_joint_values = head_group_->getCurrentJointValues();
	std::vector<std::string> joint_names = head_group_->getJoints();

	ROS_ASSERT(current_joint_values.size() == joint_names.size());
	for (size_t i = 0; i < current_joint_values.size(); i++)
	{
		if (joint_names[i] == joint_name_head_yaw_)
			pointed_head_joint_value_ = current_joint_values[i];
	}

	return true;
}

bool ActionExecutorInspectLocation::executeUpdatePlanningSceneFromORK(bool verify_planning_scene_update,
		const std::vector<std::string>& expected_objects,
		bool add_tables,
		std::string table_prefix,
		bool merge_tables)
{
	// execute action ork-to-planning-scene
	ROS_INFO("ActionExecutorInspectLocation::%s: Sending UpdatePlanningSceneFromORK request.", __func__);
	ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoal updatePSGoal;
	updatePSGoal.verify_planning_scene_update = verify_planning_scene_update;
	updatePSGoal.expected_objects = expected_objects;
	updatePSGoal.add_tables = add_tables;
	updatePSGoal.table_prefix = table_prefix;
	updatePSGoal.merge_tables = merge_tables;

	ROS_INFO("ActionExecutorInspectLocation::%s: number of expected_objects: %lu", __func__, expected_objects.size());

	actionOrkToPs_->sendGoal(updatePSGoal);
	bool finished_before_timeout = actionOrkToPs_->waitForResult(actionTimeOut_);
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionOrkToPs_->getState();
		ROS_DEBUG("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action finished: %s", __func__, state.toString().c_str());
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action succeeded.", __func__);
			ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResultConstPtr result = actionOrkToPs_->getResult();
			std::vector<std::string> seen_objects = result->added_objects;
			seen_objects.insert(seen_objects.end(), result->moved_objects.begin(), result->moved_objects.end());
			for (size_t i = 0; i < seen_objects.size(); i++)
			{
				if (expected_objects_.erase(seen_objects.at(i)) > 0)
					ROS_INFO("ActionExecutorInspectLocation::%s: Object %s has been detected, no longer an expected object",
							__func__, seen_objects.at(i).c_str());
			}
			return true;
		}
		else
		{
			ROS_ERROR("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action failed.", __func__);
			return false;
		}
	}
	ROS_ERROR("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action timed out.", __func__);
	return false;
}

bool ActionExecutorInspectLocation::executeTurnHead(const int degrees)
{
	ROS_INFO("ActionExecutorInspectLocation::%s: turn head by %d degrees.", __func__, degrees);
	moveit::planning_interface::MoveItErrorCode error_code;
	// convert degree to radians
	double radians = angles::from_degrees(degrees);

	std::vector<double> current_joint_values = head_group_->getCurrentJointValues();
	std::vector<std::string> joint_names = head_group_->getJoints();

	ROS_ASSERT(current_joint_values.size() == joint_names.size());
	// create map needed by setJointValueTarget() containing both head joints
	std::map<std::string, double> jointValues;
	for (size_t i = 0; i < current_joint_values.size(); i++)
	{
		std::pair<std::string, double> jointValue;
		if (joint_names[i] == joint_name_head_yaw_)
			jointValue = std::make_pair(joint_names[i], pointed_head_joint_value_ + radians);
		else
			jointValue = std::make_pair(joint_names[i], current_joint_values[i]);

		ROS_DEBUG("ActionExecutorInspectLocation::%s: %s - %lf - old value: %lf", __func__, jointValue.first.c_str(),
				jointValue.second, current_joint_values[i]);
		jointValues.insert(jointValue);
	}

	if (!head_group_->setJointValueTarget( jointValues ))
		ROS_ERROR("ActionExecutorInspectLocation::%s: joint values out of bounds!", __func__);
	error_code = head_group_->move();

	return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

void ActionExecutorInspectLocation::renameTableCollisionObject(const std::string& tableName)
{
	moveit_msgs::CollisionObject tableCO;
	forEach (const moveit_msgs::CollisionObject& co, psi_->getCollisionObjects())
	{
		if (StringUtil::startsWith(co.id, tableName))
		{
			ROS_INFO("ActionExecutorInspectLocation::%s: Found object with id: %s, renaming to: %s", __func__, co.id.c_str(), tableName.c_str());
			tableCO = co;
			psi_->renameCollisionObject(tableName, tableCO);
		}
	}
}

}; // namespace
