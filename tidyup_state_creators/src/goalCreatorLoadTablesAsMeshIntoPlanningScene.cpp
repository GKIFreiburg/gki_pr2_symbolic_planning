/*
 *
 *  Created on: Feb 8, 2015
 *      Author: Lanners Luc
 *
 */

#include "tidyup_state_creators/goalCreatorLoadTablesAsMeshIntoPlanningScene.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_tools/solid_primitive_dims.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorLoadTablesAsMeshIntoPlanningScene, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{
	GoalCreatorLoadTablesAsMeshIntoPlanningScene::GoalCreatorLoadTablesAsMeshIntoPlanningScene()
	{
		ros::NodeHandle nh;
		pubPlanningScene_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

	    ROS_INFO("GoalCreatorLoadTablesAsMeshIntoPlanningScene::%s: Waiting for planning_scene publisher "
	    		"to have subscribers.", __func__);
	    while(pubPlanningScene_.getNumSubscribers() < 1) {
	        ros::Duration(0.5).sleep();
	    }
	}

	GoalCreatorLoadTablesAsMeshIntoPlanningScene::~GoalCreatorLoadTablesAsMeshIntoPlanningScene()
	{
	}

	void GoalCreatorLoadTablesAsMeshIntoPlanningScene::initialize(const std::deque<std::string> & arguments)
	{
		ROS_ASSERT(arguments.size() == 1);
		type_table_ = arguments[0];
	}

	bool GoalCreatorLoadTablesAsMeshIntoPlanningScene::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
	{
		std::vector<TableLocation> tables;
		if (!symbolic_planning_utils::LoadTables::getTables(tables))
		{
			ROS_ERROR("GoalCreatorLoadTablesAsMeshIntoPlanningScene::%s: Could not load tables", __func__);
			return false;
		}
		loadTablesIntoPlanningScene(tables);

		// Add tables to current state.
		forEach(const TableLocation& tl, tables)
		{
			const string& tablename = tl.name;
			currentState.addObject(tablename, type_table_);
			currentState.setNumericalFluent("timestamp", tablename, tl.pose.header.stamp.toSec());
            currentState.addObject(tl.pose.header.frame_id, "frameid");
            currentState.setObjectFluent("frame-id", tablename, tl.pose.header.frame_id);
            currentState.setNumericalFluent("x", tablename, tl.pose.pose.position.x);
            currentState.setNumericalFluent("y", tablename, tl.pose.pose.position.y);
            currentState.setNumericalFluent("z", tablename, tl.pose.pose.position.z);
            currentState.setNumericalFluent("qx", tablename, tl.pose.pose.orientation.x);
            currentState.setNumericalFluent("qy", tablename, tl.pose.pose.orientation.y);
            currentState.setNumericalFluent("qz", tablename, tl.pose.pose.orientation.z);
            currentState.setNumericalFluent("qw", tablename, tl.pose.pose.orientation.w);
		}

		return true;
	}

	void GoalCreatorLoadTablesAsMeshIntoPlanningScene::loadTablesIntoPlanningScene(const std::vector<TableLocation>& tables)
	{
		ros::NodeHandle nhPriv("~");
	    moveit_msgs::PlanningScene planning_scene;
	    planning_scene.is_diff = true;

		forEach(const TableLocation& tl, tables)
		{
			std::string param = "table_meshes/" + tl.name;
			std::string param_package = param + "/package";
			std::string package;
			if (!nhPriv.getParam(param_package, package))
			{
				ROS_ERROR("GoalCreatorLoadTablesAsMeshIntoPlanningScene::%s: Could not load mesh for table %s", __func__, tl.name.c_str());
				continue;
			}
			package = ros::package::getPath(package);

			std::string param_path = param + "/path";
			std::string path;
			if (!nhPriv.getParam(param_path, path))
			{
				ROS_ERROR("GoalCreatorLoadTablesAsMeshIntoPlanningScene::%s: Could not load mesh for table %s", __func__, tl.name.c_str());
				continue;
			}

			std::string file_name = package + "/" + path;
			ROS_INFO("GoalCreatorLoadTablesAsMeshIntoPlanningScene::%s: Path to table mesh: %s", __func__, file_name.c_str());

			rosbag::Bag bag;
			bag.open(file_name, rosbag::bagmode::Read);

			std::string topic = "export_table";

			rosbag::View view(bag, rosbag::TopicQuery(topic));
			moveit_msgs::CollisionObject obj;
			foreach(rosbag::MessageInstance const m, view)
			{
				moveit_msgs::CollisionObject::ConstPtr co = m.instantiate<moveit_msgs::CollisionObject>();
				if (co != NULL)
				{
					obj = *co;
					obj.operation = moveit_msgs::CollisionObject::ADD;
				}
			}
			planning_scene.world.collision_objects.push_back(obj);

			// Set colors
			moveit_msgs::ObjectColor oc;
			oc.id = obj.id;
            oc.color.r = 0.67;
            oc.color.g = 0.33;
            oc.color.b = 0.0;
            oc.color.a = 1.0;
            planning_scene.object_colors.push_back(oc);
			bag.close();
		}
		pubPlanningScene_.publish(planning_scene);
	}

};


