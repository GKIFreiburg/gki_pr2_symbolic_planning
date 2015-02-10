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

			void setInitialScene(const moveit_msgs::PlanningScene& scene);
			void initializePlanningScene();

			virtual void initialize(const std::deque<std::string> & arguments);
			virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);

		private:

			void initializeTables(const SymbolicState & currentState);
			void findMatchingTable(SymbolicState & currentState, const string& object, const geometry_msgs::Pose& pose);

			moveit_msgs::PlanningScene initial_scene_;
			std::set<string> tables_;
			std::multimap<string, string> tableLocations_; // <table, location>
			typedef multimap<string,string>::const_iterator TableLocationsIterator;
	};

};

#endif

