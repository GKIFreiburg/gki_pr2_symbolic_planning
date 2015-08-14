#include "tidyup_state_creators/stateCreatorRobotPoseGrounding.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>
#include <symbolic_planning_utils/extractPose.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorRobotPoseGrounding,
		continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{

StateCreatorRobotPoseGrounding::StateCreatorRobotPoseGrounding()
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

StateCreatorRobotPoseGrounding::~StateCreatorRobotPoseGrounding()
{
}

void StateCreatorRobotPoseGrounding::initialize(
		const std::deque<std::string> & arguments)
{
	ROS_ASSERT(arguments.size() == 4);

	robot_x_ 				   = arguments[0]; 		// robot-x
	robot_y_				   = arguments[1];   	// robot-y
	robot_theta_ 			   = arguments[2];     	// robot-theta
	prediate_robot_near_table_ = arguments[3];    	// robot-near-table

}

bool StateCreatorRobotPoseGrounding::fillState(SymbolicState & state)
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

	// 1. Update real robot pose
	state.setNumericalFluent(robot_x_, "", transform.getOrigin().x());
	state.setNumericalFluent(robot_y_, "", transform.getOrigin().y());
	state.setNumericalFluent(robot_theta_, "", tf::getYaw(transform.getRotation()));

	// 2. Set robot-near-table predicate if appropriate
	// if robot is closer than 3m from table, than set predicate
	// TODO: use inv reach maps!
	string table_name;
	geometry_msgs::PoseStamped table_pose;
	double table_radius = 1.0;

	pair<SymbolicState::TypedObjectConstIterator,
			SymbolicState::TypedObjectConstIterator> targets =
			state.getTypedObjects().equal_range("table");
	for (SymbolicState::TypedObjectConstIterator it = targets.first;
			it != targets.second; it++)
	{
		// it->first = objectType, it->second = ObjectName
		table_name = it->second;

		// fetch table pose from symbolic state
		if (!symbolic_planning_utils::extractPoseStampedFromSymbolicState(state,
				table_name, table_pose))
		{
			ROS_ERROR("%s: could not extract pose for target object: %s.",
					__func__, table_name.c_str());
			continue;
		}

		// FIXME: replace with ros_assertion?
		if (table_pose.header.frame_id != "/map")
		{
			ROS_ERROR("Target pose %s had frame-id: %s - should be /map.",
					table_name.c_str(), table_pose.header.frame_id.c_str());
			continue;
		}

		// compute dXY between current pose and table
		tf::Transform targetTransform; //(btQuaternion(qx, qy, qz, qw), btVector3(posX, posY, 0.0));
		tf::poseMsgToTF(table_pose.pose, targetTransform);
		tf::Transform deltaTransform = targetTransform.inverseTimes(transform);

		double dDist = hypot(deltaTransform.getOrigin().x(),
				deltaTransform.getOrigin().y());

		if (dDist < table_radius)
		{
			ROS_INFO("Robot is near table: %s !", table_name.c_str());
			state.setBooleanPredicate(prediate_robot_near_table_, table_name, true);
		}
	}

	return true;
}

}; // namespace tidyup_state_creators

