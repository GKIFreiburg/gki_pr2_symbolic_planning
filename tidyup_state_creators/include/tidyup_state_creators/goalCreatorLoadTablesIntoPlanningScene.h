/*
 *
 *  Created on: Feb 8, 2015
 *      Author: Lanners Luc
 *
 */

#ifndef GOALCREATORLOADTABLESINTOPLANNINGSCENE_H_
#define GOALCREATORLOADTABLESINTOPLANNINGSCENE_H_

#include "continual_planning_executive/goalCreator.h"
#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <symbolic_planning_utils/load_tables.h>

namespace tidyup_state_creators
{
	typedef symbolic_planning_utils::LoadTables::TableLocation TableLocation;

	class GoalCreatorLoadTablesIntoPlanningScene: public continual_planning_executive::GoalCreator
	{
		public:

			GoalCreatorLoadTablesIntoPlanningScene();
			~GoalCreatorLoadTablesIntoPlanningScene();

			virtual void initialize(const std::deque<std::string> & arguments);
			virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);

		private:

			string type_table_;
			ros::Publisher pubPlanningScene_;

			// Create collision objects and publish them into the planning scene
			void loadTablesIntoPlanningScene(const std::vector<TableLocation>& tables);

	};

};

#endif /* GOALCREATORLOADTABLESINTOPLANNINGSCENE_H_ */
