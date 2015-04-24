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
#include <iostream>
#include <ros/publisher.h>

namespace tidyup_state_creators
{

	class GoalCreatorLoadTablesIntoPlanningScene: public continual_planning_executive::GoalCreator
	{
		public:

			struct TableLocation {
			  std::string name;
			  geometry_msgs::PoseStamped pose;
			  float sizex, sizey, sizez;
			};

			typedef GoalCreatorLoadTablesIntoPlanningScene::TableLocation tableLocation;

			GoalCreatorLoadTablesIntoPlanningScene();
			~GoalCreatorLoadTablesIntoPlanningScene();

			virtual void initialize(const std::deque<std::string> & arguments);
			virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);

		private:

			std::vector<tableLocation> tables_;
			ros::Publisher pubPlanningScene_;

			// Load a file containing table information and store them into tables_
			bool load(const std::string& filename);
			tableLocation getTableLocationFromString(const std::string& line);

			// Get all tables (name, Pose, size)
			inline const std::vector<tableLocation>& getTables() const { return tables_; };

			// Create collision objects and publish them into the planning scene
			void loadTablesIntoPlanningScene();


			// HACK FOR TEST PURPOSES.
//				// Create imaginary collision object (=graspable object) and publish it into the planning scene
//				// Hack to test planner
//				bool loadObjectIntoPlanningScene(moveit_msgs::CollisionObject& co);
//				// Hack to test planner
//				void fillObjectIntoState(SymbolicState& currentState);
	};

};


#endif /* GOALCREATORLOADTABLESINTOPLANNINGSCENE_H_ */
