#ifndef LOAD_TABLES_H_
#define LOAD_TABLES_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace symbolic_planning_utils
{

	class LoadTables
	{
		public:

			struct TableLocation
			{
			  std::string name;
			  geometry_msgs::PoseStamped pose;
			  float sizex, sizey, sizez;
			};

			// Get all tables (name, Pose, size) from tables.dat file
			static bool getTables(std::vector<TableLocation>& tables);

		private:
			// Load a file containing table information and store them into tables_
			static bool load(const std::string& filename, std::vector<TableLocation>& tables);
			// Parse line and create a TableLocation object
			static TableLocation getTableLocationFromString(const std::string& line);

	};

};

#endif /* LOAD_TABLES_H_ */
