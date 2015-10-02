/*
 *
 *  Created on: Feb 8, 2015
 *      Author: Lanners Luc
 *
 */

#ifndef GOAL_CREATOR_LOAD_TABLES_AS_MESH_INTO_PLANNING_SCENE_H_
#define GOAL_CREATOR_LOAD_TABLES_AS_MESH_INTO_PLANNING_SCENE_H_

#include "continual_planning_executive/goalCreator.h"
#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <symbolic_planning_utils/load_tables.h>

namespace tidyup_state_creators
{
	typedef symbolic_planning_utils::LoadTables::TableLocation TableLocation;

	class GoalCreatorLoadTablesAsMeshIntoPlanningScene: public continual_planning_executive::GoalCreator
	{
		public:

			GoalCreatorLoadTablesAsMeshIntoPlanningScene();
			~GoalCreatorLoadTablesAsMeshIntoPlanningScene();

			virtual void initialize(const std::deque<std::string> & arguments);
			virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);

		private:

			string type_table_;
			ros::Publisher pubPlanningScene_;

			// Create collision objects and publish them into the planning scene
			void loadTablesIntoPlanningScene(const std::vector<TableLocation>& tables);

	};

};

#endif // GOAL_CREATOR_LOAD_TABLES_AS_MESH_INTO_PLANNING_SCENE_H_
