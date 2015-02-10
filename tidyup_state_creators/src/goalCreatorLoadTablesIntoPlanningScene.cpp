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
		pub_co_ = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);

		ros::WallDuration(0.5).sleep();
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




		// using hack
		fillObjectIntoState(currentState);


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
		forEach(const tableLocation& tl, getTables())
		{
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
			pub_co_.publish(co);
		}
	}

	bool GoalCreatorLoadTablesIntoPlanningScene::loadObjectIntoPlanningScene(moveit_msgs::CollisionObject& co)
	{
		if (tables_.size() == 0)
		{
			ROS_WARN("GoalCreatorLoadTablesIntoPlanningScene::%s: No tables were loaded, so no object created", __func__);
			return false;
		}

		co.header.stamp = ros::Time::now();
		co.header.frame_id = tables_[0].pose.header.frame_id;

		co.id = "coke";
		co.primitives.resize(1);
		co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.067;
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.067;
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.12;

		// load pose of table[0] and add 20 cm to z coordinate
		// object hover over table[0]
		geometry_msgs::Pose poseCoke = tables_[0].pose.pose;
		poseCoke.position.z += 0.2;
		co.primitive_poses.push_back(poseCoke);

		co.operation = co.ADD;
		pub_co_.publish(co);
		return true;
	}

	void GoalCreatorLoadTablesIntoPlanningScene::fillObjectIntoState(SymbolicState& currentState)
	{
		moveit_msgs::CollisionObject co;
		if (!loadObjectIntoPlanningScene(co))
		{
			ROS_WARN("GoalCreatorLoadTablesIntoPlanningScene::%s: Failed to load Object into Planning Scene", __func__);
			return;
		}
		geometry_msgs::Pose poseCoke;
		const string& objectname = co.id;
		currentState.addObject(objectname, "movable_object");
		currentState.setNumericalFluent("timestamp", objectname, co.header.stamp.toSec());
        currentState.addObject(co.header.frame_id, "frameid");
        currentState.setObjectFluent("frame-id", objectname, co.header.frame_id);
        currentState.setNumericalFluent("x", objectname, co.primitive_poses[0].position.x);
        currentState.setNumericalFluent("y", objectname, co.primitive_poses[0].position.y);
        currentState.setNumericalFluent("z", objectname, co.primitive_poses[0].position.z);
        currentState.setNumericalFluent("qx", objectname, co.primitive_poses[0].orientation.x);
        currentState.setNumericalFluent("qy", objectname, co.primitive_poses[0].orientation.y);
        currentState.setNumericalFluent("qz", objectname, co.primitive_poses[0].orientation.z);
        currentState.setNumericalFluent("qw", objectname, co.primitive_poses[0].orientation.w);

        currentState.setBooleanPredicate("object-on", objectname + " " + tables_[0].name, "true");

	}
};


