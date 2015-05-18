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
ActionExecutorInspectLocation::ActionExecutorInspectLocation() :
		actionLiftTorso_("torso_controller/position_joint_action", true),
		actionPointHead_("head_traj_controller/point_head_action", true),
		actionOrkToPs_("ork_to_planning_scene", true)
{
	actionTimeOut_ = ros::Duration(30.0);
	torsoPosition_ = -1.0;
	setTorsoPosition_ = false;
	joint_name_head_yaw_ = "head_pan_joint";
	head_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getHeadGroup();

	ROS_INFO("ActionExecutorInspectLocation::%s: Waiting for torso_controller/position_joint_action "
			"action server to start.", __func__);
	actionLiftTorso_.waitForServer();

	ROS_INFO("ActionExecutorInspectLocation::%s: Waiting for head_traj_controller/point_head_action "
			"action server to start.", __func__);
	actionPointHead_.waitForServer();

	ROS_INFO("ActionExecutorInspectLocation::%s: Waiting for ork_to_planning_scene "
			"action server to start.", __func__);
	actionOrkToPs_.waitForServer();

	ROS_INFO("ActionExecutorInspectLocation::%s: Action clients are ready", __func__);

    ros::NodeHandle nhPriv("~");
    // Namespace is "/continual_planning_executive"(/vdist_head_to_table)
    nhPriv.param("vdist_head_to_table", vdist_head_to_table_, 0.8);
    nhPriv.param("vdist_threshold", vdist_threshold_, 0.002);
    nhPriv.param("min_torso_vel", min_torso_vel_, 0.0001);
    nhPriv.param("stallThreshold", stallThreshold_, 2);
    nhPriv.param("degrees", degrees_, 30);

	psi_.reset(new symbolic_planning_utils::PlanningSceneMonitor());
	//psi_.reset(new symbolic_planning_utils::PlanningSceneService());
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

//	if (!executeLiftTorso(tablePose))
//		return false;

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
			expected_objects.push_back(it->second);
		}
	}

	// sleep is needed before each object detection, so that the camera has chance to deliver good images
	ros::Duration(1.0).sleep();
	if (!executeUpdatePlanningSceneFromORK(verify_planning_scene_update, expected_objects,
			add_tables, table_prefix, merge_tables))
		return false;

	// turn head by given degrees - return value is ignored, since action is executed but reports error
	if (!executeTurnHead(degrees_))
		return false;

	// execute visual detection to merge table
	merge_tables = true;
	ros::Duration(1.0).sleep();
	if (!executeUpdatePlanningSceneFromORK(verify_planning_scene_update, expected_objects,
			add_tables, table_prefix, merge_tables))
		return false;

	// turn head by given degrees, this time in opposite direction
	if (!executeTurnHead(degrees_ * -1))
		return false;

	// execute visual detection to merge table
	merge_tables = true;
	ros::Duration(1.0).sleep();
	if (!executeUpdatePlanningSceneFromORK(verify_planning_scene_update, expected_objects,
			add_tables, table_prefix, merge_tables))
		return false;

	// After operation is done, point head again to table
	if (!executePointHead(tablePose))
		return false;

	// after detection is completed, set predicates: location-inspected and location_inspected-recently
	for_each(const string& predicate, predicate_names_)
	{
		currentState.setBooleanPredicate(predicate, mani_loc, true);
	}

	// Cut off _number from table detection and readd table to PS
	renameTableCollisionObject(table);

	return true;
}

void ActionExecutorInspectLocation::cancelAction()
{
}

void ActionExecutorInspectLocation::feedbackLiftTorso(const control_msgs::SingleJointPositionFeedbackConstPtr& feedback)
{
	if (setTorsoPosition_)
	{
		torsoPosition_ = feedback->position;
		actionLiftTorso_.cancelGoal();
		setTorsoPosition_ = false;
		return;
	}

	if (fabs(feedback->velocity) < min_torso_vel_)
	{
		if (startStallTime_ == 0)
			startStallTime_ = ros::Time::now().toSec();
		// waited longer than stallThreshold_ -> torso is stalled
		else if ((ros::Time::now().toSec() - startStallTime_) > stallThreshold_)
		{
			ROS_WARN("ActionExecutorInspectLocation::%s: Torso stalled - limit reached (Position: %lf, Velocity: %lf)",
					__func__, feedback->position, feedback->velocity);
			actionLiftTorso_.cancelGoal();
			control_msgs::SingleJointPositionGoal liftTorsoGoal;
			liftTorsoGoal.position = feedback->position;
			actionLiftTorso_.sendGoal(liftTorsoGoal);
		}
	}

	ROS_DEBUG_STREAM_THROTTLE(0.10, "Torso Joint Position: " << feedback->position << " , Velocity: " << feedback->velocity << " , Error: " << feedback->error);

}

bool ActionExecutorInspectLocation::executeLiftTorso(const geometry_msgs::PoseStamped tablePose)
{
	// compute difference in height between head_mount_link and table
    // transform tablePose into frame of /base link
    geometry_msgs::PoseStamped table_transformed;
    try {
        tf_.waitForTransform("/base_link", tablePose.header.frame_id, tablePose.header.stamp,
                ros::Duration(0.5));
        tf_.transformPose("/base_link", tablePose, table_transformed);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    ROS_DEBUG_STREAM("table pose" << tablePose);
    ROS_DEBUG_STREAM("table pose transformed" << table_transformed);

    // transform headPose into frame of /base link
    tf::Pose tfHead;
    geometry_msgs::PoseStamped headPose;
    headPose.header.frame_id = "/head_mount_link";
    headPose.header.stamp = ros::Time::now();
    geometry_msgs::Quaternion q;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    q.w = 1;
    headPose.pose.orientation = q;
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    headPose.pose.position = p;

    geometry_msgs::PoseStamped head_transformed;
    try {
        tf_.waitForTransform("/map", headPose.header.frame_id, headPose.header.stamp,
                ros::Duration(0.5));
        tf_.transformPose("/map", headPose, head_transformed);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    ROS_DEBUG_STREAM("head pose transformed" << head_transformed);

    double distance = fabs(head_transformed.pose.position.z - table_transformed.pose.position.z);
    ROS_DEBUG("Distance: %lf, head: %lf, table: %lf", distance, head_transformed.pose.position.z, table_transformed.pose.position.z);
    if (vdist_head_to_table_ - vdist_threshold_ < distance &&
    	distance < vdist_head_to_table_ + vdist_threshold_)
    	// head_mount_link is about vdist_head_to_table_ above table, no need to lift torso
    	return true;

    setTorsoPosition();

	ROS_INFO("ActionExecutorInspectLocation::%s: Sending LiftTorso request.", __func__);
	control_msgs::SingleJointPositionGoal liftTorsoGoal;
	liftTorsoGoal.position = torsoPosition_ + vdist_head_to_table_ - distance;

	ROS_DEBUG_STREAM("Lift torso about : " << torsoPosition_ << " + " << vdist_head_to_table_ << " - " << distance <<
			" = " << torsoPosition_ + vdist_head_to_table_ - distance);

	startStallTime_ = 0;
	//actionLiftTorso_.sendGoal(liftTorsoGoal);
	// Checking if torso is stalled, if so cancel action and start new action with current position
	// Then actionLiftTorso will end immediately with success
	actionLiftTorso_.sendGoal(liftTorsoGoal,
		actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction>::SimpleDoneCallback(), // = NULL
		actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction>::SimpleActiveCallback(), // = NULL
		boost::bind(&ActionExecutorInspectLocation::feedbackLiftTorso, this, _1));

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

void ActionExecutorInspectLocation::setTorsoPosition()
{
	ROS_INFO("ActionExecutorInspectLocation::%s: Sending LiftTorso request.", __func__);
	control_msgs::SingleJointPositionGoal liftTorsoGoal;
	liftTorsoGoal.position = 0.32;
	setTorsoPosition_ = true;

	// http://library.isr.ist.utl.pt/docs/roswiki/actionlib_tutorials%282f%29Tutorials%282f%29Writing%2820%29a%2820%29Callback%2820%29Based%2820%29Simple%2820%29Action%2820%29Client.html
//	actionLiftTorso_.sendGoal(liftTorsoGoal,
//		boost::bind(&ActionExecutorInspectLocation::doneLiftTorso, this, _1, _2),
//		actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction>::SimpleActiveCallback(),
//		actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction>::SimpleFeedbackCallback());

	actionLiftTorso_.sendGoal(liftTorsoGoal,
		actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction>::SimpleDoneCallback(), // = NULL
		actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction>::SimpleActiveCallback(), // = NULL
		boost::bind(&ActionExecutorInspectLocation::feedbackLiftTorso, this, _1));

	actionLiftTorso_.waitForResult(actionTimeOut_);
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

	actionOrkToPs_.sendGoal(updatePSGoal);
	bool finished_before_timeout = actionOrkToPs_.waitForResult(actionTimeOut_);
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionOrkToPs_.getState();
		ROS_DEBUG("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action finished: %s", __func__, state.toString().c_str());
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("ActionExecutorInspectLocation::%s: ORK to Planning Scene Action succeeded.", __func__);
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
	// create map needed by setJointValueTarget()
	std::map<std::string, double> jointValues;
	for (size_t i = 0; i < current_joint_values.size(); i++)
	{
		std::pair<std::string, double> jointValue;
		if (joint_names[i] == joint_name_head_yaw_)
			jointValue = std::make_pair(joint_names[i], radians);
		else
			jointValue = std::make_pair(joint_names[i], current_joint_values[i]);

		ROS_WARN("ActionExecutorInspectLocation::%s: %s - %lf - old value: %lf", __func__, jointValue.first.c_str(),
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
