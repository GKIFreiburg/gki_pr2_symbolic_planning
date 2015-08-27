/*
 * inverse_reachability_maps.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: andreas
 */

#include "planner_modules_pr2/inverse_reachability_maps.h"
#include <symbolic_planning_utils/load_tables.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

namespace planner_modules_pr2
{

InverseReachabilityMapsPtr InverseReachabilityMaps::instance_;

InverseReachabilityMapsPtr InverseReachabilityMaps::instance()
{
	if (instance_ == NULL)
	{
		instance_.reset(new InverseReachabilityMaps());
	}
	return instance_;
}

InverseReachabilityMaps::InverseReachabilityMaps()
{
	ros::NodeHandle nhPriv("~");
	ros::NodeHandle nh;

	// first load tables from tables.dat file
	// load table pose
	std::vector<symbolic_planning_utils::LoadTables::TableLocation> tables;
	if (!symbolic_planning_utils::LoadTables::getTables(tables))
	{
		ROS_ERROR("drive_pose_module::%s: Could not load tables!", __func__);
		return;
	}

	// store table names and poses into global variable
	forEach (const symbolic_planning_utils::LoadTables::TableLocation& table, tables)
	{
		const std::string& tableName = table.name;
		// read path to inverse reachability maps from param
		std::string package, relative_path;
		if (!nh.getParam("continual_planning_executive/inverse_reachability_maps/" + table.name + "/package", package))
		{
			ROS_ERROR_STREAM(__func__<<": Could not load package for surface: "<<table.name);
			continue;
		}
		if (!nh.getParam("continual_planning_executive/inverse_reachability_maps/" + table.name + "/path", relative_path))
		{
			ROS_ERROR_STREAM(__func__<<": Could not load relative path for surface: "<<table.name);
			continue;
		}
		std::string pkg_path = ros::package::getPath(package);
		std::string path = pkg_path + "/" + relative_path;
		// ROS_INFO("path to inv_reach: %s", path.c_str());

		// store inverse reachability maps
		boost::shared_ptr<InverseCapabilityOcTree> inverse_reachability(InverseCapabilityOcTree::readFile(path));
		TableInverseReachabilityConstPtr tableInfo(new TableInverseReachability(table.pose, inverse_reachability));
		map.insert(std::make_pair(table.name, tableInfo));
	}
}

InverseReachabilityMaps::~InverseReachabilityMaps()
{

}

} /* namespace planner_modules_pr2 */
