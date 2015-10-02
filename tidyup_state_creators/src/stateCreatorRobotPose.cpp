#include "tidyup_state_creators/stateCreatorRobotPose.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>
#include <symbolic_planning_utils/extractPose.h>
#include <symbolic_planning_utils/load_tables.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <ros/package.h>

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
			ROS_WARN("nav_target_tolerance_relative_to_move_base set, but nav_base_local_planner_ns not set - trying to estimate");
			std::string local_planner;
			if (!nh.getParam("move_base_node/base_local_planner", local_planner) && !nh.getParam("move_base/base_local_planner", local_planner))
			{
				ROS_ERROR("move_base(_node)/base_local_planner not set - falling back to absolute mode.");
			}
			else
			{
				// dwa_local_planner/DWAPlannerROS -> DWAPlannerROS
				std::string::size_type x = local_planner.find_last_of("/");
				if (x == std::string::npos)
					base_local_planner_ns = local_planner;
				else
					base_local_planner_ns = local_planner.substr(x + 1);
				ROS_INFO("Estimated base_local_planner_ns to %s.", base_local_planner_ns.c_str());
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

	// LOAD Inverse Capability Maps
    // first load tables from tables.dat file
	// load table pose
	std::vector<symbolic_planning_utils::LoadTables::TableLocation> tables;
	if (!symbolic_planning_utils::LoadTables::getTables(tables))
	{
		ROS_ERROR("StateCreatorRobotPose::%s: Could not load tables", __func__);
		return;
	}

	// store table names and the corresponding inv_reach_map into a member variable
	for (size_t i = 0; i < tables.size(); i++)
	{
		const std::string& tableName = tables[i].name;

		// read path to inverse reachability maps from param
		std::string package, relative_path;
		if (!nh.getParam("continual_planning_executive/inverse_reachability_maps/" + tableName + "/package", package))
		{
			ROS_ERROR("StateCreatorRobotPose::%s: Could not load package for surface: %s", __func__, tableName.c_str());
			continue;
		}
		if (!nh.getParam("continual_planning_executive/inverse_reachability_maps/" + tableName + "/path", relative_path))
		{
			ROS_ERROR("StateCreatorRobotPose::%s: Could not load relative path for surface: %s", __func__, tableName.c_str());
			continue;
		}

		std::string pkg_path = ros::package::getPath(package);
		std::string path = pkg_path + "/" + relative_path;

		// store inverse capability maps into global variable
		InverseCapabilityOcTree* tree = InverseCapabilityOcTree::readFile(path);
		std::pair<std::string, InverseCapabilityOcTree*> icm = std::make_pair(tableName, tree);
		inv_cap_maps_.insert(icm);
	}

}

StateCreatorRobotPose::~StateCreatorRobotPose()
{
}

void StateCreatorRobotPose::initialize(
		const std::deque<std::string> & arguments)
{
	ROS_ASSERT(arguments.size() == 5);

	robot_x_ 				   = arguments[0]; 		// robot-x
	robot_y_				   = arguments[1];   	// robot-y
	robot_theta_ 			   = arguments[2];     	// robot-theta
	robot_torso_position_      = arguments[3];		// robot-torso-position
	prediate_robot_near_table_ = arguments[4];    	// robot-near-table

	torso_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getTorsoGroup();

}

bool StateCreatorRobotPose::fillState(SymbolicState & state)
{
	tf::StampedTransform transform;
	try
	{
		tf_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	} catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
		return false;
	}

	// 1. Update real robot pose
	state.setNumericalFluent(robot_x_, "", transform.getOrigin().x());
	state.setNumericalFluent(robot_y_, "", transform.getOrigin().y());
	state.setNumericalFluent(robot_theta_, "", tf::getYaw(transform.getRotation()));
	const std::vector<double>& jointValues = torso_group_->getCurrentJointValues();
	ROS_ASSERT(jointValues.size() == 1);
	state.setNumericalFluent(robot_torso_position_, "", jointValues[0]);

//	// 2. Set robot-near-table predicate if appropriate
//	string table_name;
//	geometry_msgs::PoseStamped table_pose;
//	tf::StampedTransform transform_map_torso;
//	try
//	{
//		tf_.lookupTransform("/map", "/torso_lift_link", ros::Time(0), transform_map_torso);
//	} catch (tf::TransformException& ex)
//	{
//		ROS_ERROR("%s", ex.what());
//		return false;
//	}
//
//	pair<SymbolicState::TypedObjectConstIterator,
//			SymbolicState::TypedObjectConstIterator> targets =
//			state.getTypedObjects().equal_range("table");
//	for (SymbolicState::TypedObjectConstIterator it = targets.first;
//			it != targets.second; it++)
//	{
//		// it->first = objectType, it->second = ObjectName
//		table_name = it->second;
//
//		// fetch table pose from symbolic state
//		if (!symbolic_planning_utils::extractPoseStampedFromSymbolicState(state,
//				table_name, table_pose))
//		{
//			ROS_ERROR("StateCreatorRobotPose::%s: could not extract pose for target object: %s.",
//					__func__, table_name.c_str());
//			continue;
//		}
//
//		ROS_ASSERT(table_pose.header.frame_id == "/map");
//		if (inv_cap_maps_.find(table_name) == inv_cap_maps_.end())
//		{
//			ROS_ERROR_STREAM("StateCreatorRobotPose::"<<__func__<<": could not load inverse reachability map for "<<table_name);
//			continue;
//		}
//		InverseCapabilityOcTree* tree = inv_cap_maps_[table_name];
//		ROS_ASSERT(tree);
//
//		// fetch torso pose and convert into table frame
//		tf::Pose transform_map_table, transform_table_torso;
//		tf::poseMsgToTF(table_pose.pose, transform_map_table);
//
//		transform_table_torso = transform_map_table.inverseTimes(transform_map_torso);
////		tf::Pose torso_table = transformTorsoInTableFrame(table_pose);
//
////		geometry_msgs::Pose debug;
////		tf::poseTFToMsg(torso_table, debug);
////		ROS_INFO_STREAM("StateCreatorRobotPose::" << __func__ << ": " << debug);
//
//		// look if there is a match in inv cap map
//		InverseCapability inv = tree->getNodeInverseCapability(transform_table_torso.getOrigin().x(),
//				transform_table_torso.getOrigin().y(),
//				transform_table_torso.getOrigin().z());
//
//		const std::map<double, double>& thetas = inv.getThetasPercent();
//
//		// if there exists a theta, means we have an inverse reachability index,
//		// indicating that a part of the table can be reached -> we are close to table
//		if (thetas.size() > 0)
//		{
//			ROS_INFO("StateCreatorRobotPose::%s: Robot is near '%s' !", __func__, table_name.c_str());
//			state.setBooleanPredicate(prediate_robot_near_table_, table_name, true);
//		}
//		else
//			state.setBooleanPredicate(prediate_robot_near_table_, table_name, false);
//	}

	return true;
}

tf::Pose StateCreatorRobotPose::transformTorsoInTableFrame(const geometry_msgs::PoseStamped& table)
{
	tf::StampedTransform transform_map_torso;
	try
	{
		tf_.lookupTransform("/map", "/torso_lift_link", ros::Time(0), transform_map_torso);
	} catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
	}

	tf::Pose transform_map_table, transform_table_torso;
	tf::poseMsgToTF(table.pose, transform_map_table);

	transform_table_torso = transform_map_table.inverseTimes(transform_map_torso);

	return transform_table_torso;
}

}; // namespace tidyup_state_creators

