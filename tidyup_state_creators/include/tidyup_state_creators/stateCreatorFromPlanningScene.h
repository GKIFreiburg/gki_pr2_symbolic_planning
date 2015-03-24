#ifndef STATE_CREATOR_FROM_PLANNING_SCENE_H
#define STATE_CREATOR_FROM_PLANNING_SCENE_H

#include "continual_planning_executive/stateCreator.h"
#include <set>

#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <gtest/gtest.h>

namespace tidyup_state_creators
{
	class StateCreatorFromPlanningScene: public continual_planning_executive::StateCreator
	{
		public:

			StateCreatorFromPlanningScene();
			~StateCreatorFromPlanningScene();

            /// Initialize the action from a list of arguments - should be called after creating the interface.
            virtual void initialize(const std::deque<std::string>& arguments);

            virtual bool fillState(SymbolicState& state);

		private:

            moveit_msgs::PlanningScene planningScene_;
            ros::ServiceClient srvPlanningScene_;
            std::set<string> tables_;
            std::multimap<string, string> tableLocations_; // <table, location>

            void initializePlanningScene();
            void setPlanningScene(const moveit_msgs::PlanningScene& scene);

            FRIEND_TEST(stateCreatorFromPlanningSceneTest, initializeTables);
            void initializeTables(const SymbolicState& currentState);

            FRIEND_TEST(stateCreatorFromPlanningSceneTest, addObjectToSymbolicState);
            void addObjectToSymbolicState(SymbolicState& state, const moveit_msgs::CollisionObject& co,
            		const std::string& objectType);

            // Checks if the objectType in addObjectToSymbolicState really exist
            // it is a help function to detect typos
            FRIEND_TEST(stateCreatorFromPlanningSceneTest, doesObjectTypeExist);
            bool doesObjectTypeExist(const string& objectType);

            FRIEND_TEST(stateCreatorFromPlanningSceneTest, findMatchingTable);
            void findMatchingTable(SymbolicState& currentState,
            		const std::vector<moveit_msgs::CollisionObject>& allCos,
            		const moveit_msgs::CollisionObject& co);

            FRIEND_TEST(stateCreatorFromPlanningSceneTest, extractPoseStampedFromCollisionObject);
            bool extractPoseStampedFromCollisionObject(const moveit_msgs::CollisionObject& co,
            		geometry_msgs::PoseStamped& pose) const;

	};

};

#endif

