#include "symbolic_planning_utils/load_tables.h"
#include <iostream>
#include <fstream>
#include <sstream>

namespace symbolic_planning_utils
{
	// Get all tables (name, Pose, size) from tables.dat file
	bool LoadTables::getTables(std::vector<TableLocation>& tables)
	{
        ros::NodeHandle nhPriv("~");
		// load table locations from file
		std::string tablesFile;

		if(!nhPriv.getParam("tables", tablesFile)) {
			ROS_ERROR("Could not get ~tables parameter.");
			return false;
		}

		if (!load(tablesFile, tables))
		{
			ROS_ERROR("Could not load tables from file \"%s\".", tablesFile.c_str());
			return false;
		}

		return true;
	}


	bool LoadTables::load(const std::string& filename, std::vector<TableLocation>& tables)
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
			TableLocation tl = getTableLocationFromString(line);
			tables.push_back(tl);
		}
		return true;
	}

	LoadTables::TableLocation LoadTables::getTableLocationFromString(const std::string& line)
	{
		std::stringstream ss(line);
		TableLocation tl;
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
};

