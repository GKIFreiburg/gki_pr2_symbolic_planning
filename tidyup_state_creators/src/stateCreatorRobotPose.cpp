#include "tidyup_state_creators/stateCreatorRobotPose.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>
#include <symbolic_planning_utils/extractPose.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorRobotPose,
		continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{

StateCreatorRobotPose::StateCreatorRobotPose()
{
	ros::NodeHandle nhPriv("~");
	ros::NodeHandle nh;
	nhPriv.param("nav_target_tolerance_xy", _goalToleranceXY, 0.15);
	nhPriv.param("nav_target_tolerance_yaw", _goalToleranceYaw, 0.26); // corresponds 15deg

	bool relative;
	nhPriv.param("nav_target_tolerance_relative_to_move_base", relative, false);
	if (relative)
	{
		// relative mode: 1. get the namespace for base_local_planner
		std::string base_local_planner_ns;
		if (!nhPriv.getParam("nav_base_local_planner_ns",
				base_local_planner_ns))
		{
			ROS_WARN(
					"nav_target_tolerance_relative_to_move_base set, but nav_base_local_planner_ns not set - trying to estimate");
			std::string local_planner;
			if (!nh.getParam("move_base_node/base_local_planner", local_planner)
					&& !nh.getParam("move_base/base_local_planner",
							local_planner))
			{
				ROS_ERROR(
						"move_base(_node)/base_local_planner not set - falling back to absolute mode.");
			}
			else
			{
				// dwa_local_planner/DWAPlannerROS -> DWAPlannerROS
				std::string::size_type x = local_planner.find_last_of("/");
				if (x == std::string::npos)
					base_local_planner_ns = local_planner;
				else
					base_local_planner_ns = local_planner.substr(x + 1);
				ROS_INFO("Estimated base_local_planner_ns to %s.",
						base_local_planner_ns.c_str());
			}
		}

		if (!base_local_planner_ns.empty())
		{ // success: 2. get the xy_goal_tolerance
			double move_base_tol_xy;
			if (!nh.getParam(base_local_planner_ns + "/xy_goal_tolerance",
					move_base_tol_xy))
			{
				ROS_ERROR_STREAM(
						"nav_target_tolerance_relative_to_move_base was true, but " << (base_local_planner_ns + "/xy_goal_tolerance") << " was not set" << " - falling back to absolute mode");
			}
			else
			{ // 2. add move_base's tolerance to our relative tolerance
				_goalToleranceXY += move_base_tol_xy;
			}

			double move_base_tol_yaw;
			if (!nh.getParam(base_local_planner_ns + "/yaw_goal_tolerance",
					move_base_tol_yaw))
			{
				ROS_ERROR_STREAM(
						"nav_target_tolerance_relative_to_move_base was true, but " << (base_local_planner_ns + "/yaw_goal_tolerance") << " was not set" << " - falling back to absolute mode");
			}
			else
			{ // 2. add move_base's tolerance to our relative tolerance
				_goalToleranceYaw += move_base_tol_yaw;
			}
		}
	}

	ROS_INFO("Tolerance for accepting nav goals set to %f m, %f deg.",
			_goalToleranceXY, angles::to_degrees(_goalToleranceYaw));
}

StateCreatorRobotPose::~StateCreatorRobotPose()
{
}

void StateCreatorRobotPose::initialize(
		const std::deque<std::string> & arguments)
{
	ROS_ASSERT(arguments.size() == 4);

	_robotPoseObject = arguments[0]; // robot_location
	_robotPoseType = arguments[1];   // location
	_atPredicate = arguments[2];     // robot-at
	_locationType = arguments[3];    // location

	if (_robotPoseObject == "-")
		_robotPoseObject = "";
	if (_robotPoseType == "-")
		_robotPoseType = "";
	if (_atPredicate == "-")
		_atPredicate = "";
	if (_locationType == "-")
		_locationType = "";
}

bool StateCreatorRobotPose::fillState(SymbolicState & state)
{
	tf::StampedTransform transform;
	try
	{
		_tf.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	} catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
		return false;
	}

	// 1. Real robot location
	if (!_robotPoseObject.empty())
	{
		ROS_ASSERT(!_robotPoseType.empty());
		state.addObject(_robotPoseObject, _robotPoseType);
		state.setNumericalFluent("x", _robotPoseObject,
				transform.getOrigin().x());
		state.setNumericalFluent("y", _robotPoseObject,
				transform.getOrigin().y());
		state.setNumericalFluent("z", _robotPoseObject,
				transform.getOrigin().z());
		state.setNumericalFluent("qx", _robotPoseObject,
				transform.getRotation().x());
		state.setNumericalFluent("qy", _robotPoseObject,
				transform.getRotation().y());
		state.setNumericalFluent("qz", _robotPoseObject,
				transform.getRotation().z());
		state.setNumericalFluent("qw", _robotPoseObject,
				transform.getRotation().w());
		state.setNumericalFluent("timestamp", _robotPoseObject,
				ros::Time::now().toSec());
		state.addObject("/map", "frameid");
		state.setObjectFluent("frame-id", _robotPoseObject, "/map");
	}

	// 2.b check if we are at any _locations
	pair<SymbolicState::TypedObjectConstIterator,
			SymbolicState::TypedObjectConstIterator> targets =
			state.getTypedObjects().equal_range(_locationType);

	double minDist = HUGE_VAL;
	string nearestTarget = "";

	int atLocations = 0;
	for (SymbolicState::TypedObjectConstIterator it = targets.first;
			it != targets.second; it++)
	{
		ROS_DEBUG_STREAM(
				"StateCreatorRobotPose::" << __func__ << ": ObjectType: " << it->first << " ObjectName: " << it->second);
		string target = it->second;
		if (target == _robotPoseObject)  // skip current robot location
			continue;

		geometry_msgs::PoseStamped targetPose;
		if (!symbolic_planning_utils::extractPoseStampedFromSymbolicState(state,
				target, targetPose))
		{
			ROS_ERROR("%s: could not extract pose for target object: %s.",
					__func__, target.c_str());
			continue;
		}

		// TODO: All poses should be in frame /map or?
		if (targetPose.header.frame_id != "/map")
		{
			ROS_ERROR("Target pose %s had frame-id: %s - should be /map.",
					target.c_str(), targetPose.header.frame_id.c_str());
			continue;
		}

		// compute dXY, dYaw between current pose and target
		tf::Transform targetTransform; //(btQuaternion(qx, qy, qz, qw), btVector3(posX, posY, 0.0));
		tf::poseMsgToTF(targetPose.pose, targetTransform);
		tf::Transform deltaTransform = targetTransform.inverseTimes(transform);

		double dDist = hypot(deltaTransform.getOrigin().x(),
				deltaTransform.getOrigin().y());
		double dAng = tf::getYaw(deltaTransform.getRotation());
		ROS_INFO("Target %s dist: %f m ang: %f deg", target.c_str(), dDist,
				angles::to_degrees(dAng));

		if (!_atPredicate.empty())
		{
			// Found a target - update state!
			if (dDist < minDist)
			{
				minDist = dDist;
				nearestTarget = target;
			}
			state.setBooleanPredicate(_atPredicate, target, false);
		}
	}
	if (nearestTarget != "")
	{
		ROS_INFO("(at) target %s !", nearestTarget.c_str());
		state.setBooleanPredicate(_atPredicate, nearestTarget, true);
		atLocations++;
	}

	ROS_INFO("Nearest target is %s (%f m).", nearestTarget.c_str(), minDist);

	// 2.a Set the robot pose, if we are not already at another pose
	if (!_atPredicate.empty() && !_robotPoseObject.empty())
	{
		if (atLocations == 0)
		{
			state.setBooleanPredicate(_atPredicate, _robotPoseObject, true);
		}
		else
		{
			state.setBooleanPredicate(_atPredicate, _robotPoseObject, false);
			if (atLocations > 1)
			{
				ROS_WARN("We are at %d locations at the same time!.",
						atLocations);
			}
		}
	}

	return true;
}
}
;

