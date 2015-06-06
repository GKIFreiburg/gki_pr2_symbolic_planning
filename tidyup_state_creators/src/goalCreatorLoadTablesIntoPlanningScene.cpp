/*
 *
 *  Created on: Feb 8, 2015
 *      Author: Lanners Luc
 *
 */

#include "tidyup_state_creators/goalCreatorLoadTablesIntoPlanningScene.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_tools/solid_primitive_dims.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorLoadTablesIntoPlanningScene, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{
	GoalCreatorLoadTablesIntoPlanningScene::GoalCreatorLoadTablesIntoPlanningScene()
	{
		ros::NodeHandle nh;
		pubPlanningScene_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

	    ROS_INFO("GoalCreatorLoadTablesIntoPlanningScene::%s: Waiting for planning_scene publisher "
	    		"to have subscribers.", __func__);
	    while(pubPlanningScene_.getNumSubscribers() < 1) {
	        ros::Duration(0.5).sleep();
	    }
	}

	GoalCreatorLoadTablesIntoPlanningScene::~GoalCreatorLoadTablesIntoPlanningScene()
	{
	}

	void GoalCreatorLoadTablesIntoPlanningScene::initialize(const std::deque<std::string> & arguments)
	{

	}

	bool GoalCreatorLoadTablesIntoPlanningScene::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
	{
		std::vector<TableLocation> tables;
		if (!symbolic_planning_utils::LoadTables::getTables(tables))
		{
			ROS_ERROR("GoalCreatorLoadTablesIntoPlanningScene::%s: Could not load tables", __func__);
			return false;
		}
		loadTablesIntoPlanningScene(tables);

		// Add tables to current state.
		forEach(const TableLocation& tl, tables)
		{
			const string& tablename = tl.name;
			currentState.addObject(tablename, "table");
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

	void GoalCreatorLoadTablesIntoPlanningScene::loadTablesIntoPlanningScene(const std::vector<TableLocation>& tables)
	{
	    moveit_msgs::PlanningScene planning_scene;
	    planning_scene.is_diff = true;

		forEach(const TableLocation& tl, tables)
		{
			// Create collision object for each table
			moveit_msgs::CollisionObject co;
			co.id = tl.name;
			co.header = tl.pose.header;
//			co.primitives.resize(1);
//			co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//			co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
//			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = tl.sizex;
//			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = tl.sizey;
//			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tl.sizez;
//			co.primitive_poses.push_back(tl.pose.pose);

			shapes::Box box = shapes::Box(tl.sizex, tl.sizey, tl.sizez);
			shapes::Mesh* mesh = shapes::createMeshFromShape(box);
			shapes::ShapeMsg mesh_msg;
			shapes::constructMsgFromShape(mesh, mesh_msg);

			co.meshes.resize(1);
			co.meshes[0] = boost::get<shape_msgs::Mesh>(mesh_msg);
			co.mesh_poses.push_back(tl.pose.pose);
			co.operation = co.ADD;
			planning_scene.world.collision_objects.push_back(co);

			// Set colors
			moveit_msgs::ObjectColor oc;
			oc.id = co.id;
            oc.color.r = 0.67;
            oc.color.g = 0.33;
            oc.color.b = 0.0;
            oc.color.a = 1.0;
            planning_scene.object_colors.push_back(oc);
		}
		pubPlanningScene_.publish(planning_scene);
	}

};


