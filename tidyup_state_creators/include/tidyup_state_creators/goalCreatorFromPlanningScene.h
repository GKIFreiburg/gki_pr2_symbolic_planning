#ifndef GOAL_CREATOR_FROM_PLANNING_SCENE_H
#define GOAL_CREATOR_FROM_PLANNING_SCENE_H

#include "continual_planning_executive/goalCreator.h"
#include <moveit_msgs/PlanningScene.h>
#include <set>

namespace tidyup_state_creators
{

	class GoalCreatorFromPlanningScene: public continual_planning_executive::GoalCreator
	{
		public:

			GoalCreatorFromPlanningScene();
			~GoalCreatorFromPlanningScene();

			virtual void initialize(const std::deque<std::string> & arguments);
			virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);

		private:
			// Use rosservice to fetch planning scene msg
			void initializePlanningScene();

			// set membervariable initial_scene_ with planning scene msg
			void setInitialScene(const moveit_msgs::PlanningScene& scene);

			// Fetching table and tableLocations from currentState and store them in member variables tables_
			// and tableLocations_
			void initializeTables(const SymbolicState & currentState);

			void findMatchingTable(SymbolicState & currentState, const string& object, const geometry_msgs::Pose& pose);

			moveit_msgs::PlanningScene initial_scene_;
			std::set<string> tables_;
			std::multimap<string, string> tableLocations_; // <table, location>
			typedef multimap<string,string>::const_iterator TableLocationsIterator;
	};

};

#endif

