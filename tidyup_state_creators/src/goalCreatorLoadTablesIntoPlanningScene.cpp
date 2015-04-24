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
#include <fstream>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorLoadTablesIntoPlanningScene, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{
	typedef GoalCreatorLoadTablesIntoPlanningScene::TableLocation tableLocation;

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
        ros::NodeHandle nhPriv("~");
		// load table locations from file
		std::string tablesFile;

		if(!nhPriv.getParam("tables", tablesFile)) {
			ROS_ERROR("Could not get ~tables parameter.");
			return false;
		}

		if (!load(tablesFile))
		{
			ROS_ERROR("Could not load tables from \"%s\".", tablesFile.c_str());
			return false;
		}

		loadTablesIntoPlanningScene();

		// Add tables to current state.
		forEach(const tableLocation& tl, getTables())
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

	bool GoalCreatorLoadTablesIntoPlanningScene::load(const std::string& filename)
	{
		std::ifstream f(filename.c_str());
		if(!f.good())
			return false;

		std::string line;
		while(f.good() && !f.eof()) {
			getline(f, line);
			size_t pos = line.find_first_not_of(" ");
			if(pos == std::string::npos)    // empty line
				continue;
			if(line[pos] == '#')       // comment line
				continue;
			// parse the line
			tableLocation tl = getTableLocationFromString(line);
			tables_.push_back(tl);
		}
		return true;
	}

	tableLocation GoalCreatorLoadTablesIntoPlanningScene::getTableLocationFromString
		(const std::string& line)
	{
		std::stringstream ss(line);
		tableLocation tl;
		ss >> tl.name;
		uint32_t tsec, tnsec;
		geometry_msgs::PoseStamped pose;
		ss >> tsec;
		ss >> tnsec;
		tl.pose.header.stamp = ros::Time(tsec, tnsec);
		ss >> tl.pose.header.frame_id;
		ss >> tl.pose.pose.position.x;
		ss >> tl.pose.pose.position.y;
		ss >> tl.pose.pose.position.z;
		ss >> tl.pose.pose.orientation.x;
		ss >> tl.pose.pose.orientation.y;
		ss >> tl.pose.pose.orientation.z;
		ss >> tl.pose.pose.orientation.w;
		ss >> tl.sizex;
		ss >> tl.sizey;
		ss >> tl.sizez;
		return tl;
	}

	void GoalCreatorLoadTablesIntoPlanningScene::loadTablesIntoPlanningScene()
	{
	    moveit_msgs::PlanningScene planning_scene;
	    planning_scene.is_diff = true;

		forEach(const tableLocation& tl, getTables())
		{
			// Create collision object for each table
			moveit_msgs::CollisionObject co;
			co.id = tl.name;
			co.header = tl.pose.header;
			co.primitives.resize(1);
			co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
			co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = tl.sizex;
			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = tl.sizey;
			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tl.sizez;
			co.primitive_poses.push_back(tl.pose.pose);
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


