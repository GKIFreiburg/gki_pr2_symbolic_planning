#ifndef STATE_CREATOR_FROM_PLANNING_SCENE_H
#define STATE_CREATOR_FROM_PLANNING_SCENE_H

#include "continual_planning_executive/stateCreator.h"
#include <set>

#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <tf/transform_listener.h>

namespace tidyup_state_creators
{
	class StateCreatorFromPlanningScene: public continual_planning_executive::StateCreator
	{
		public:

			StateCreatorFromPlanningScene();
			~StateCreatorFromPlanningScene();

            /// Initialize the action from a list of arguments - should be called after creating the interface.
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

		protected:

            /// Extract a PoseStamped for object from state.
            /**
             * The fluents that are queried are: x,y,z, qx,qy,qz,qw, frame-id, timestamp
             *
             * \returns true if all fluents were available
             */
            FRIEND_TEST(stateCreatorFromPlanningSceneTest, extractPoseStampedFromSymbolicState);
            bool extractPoseStampedFromSymbolicState(const SymbolicState & state, const string & object,
                    geometry_msgs::PoseStamped & pose) const;

            // the same as in ork_to_planning_scene
            FRIEND_TEST(stateCreatorFromPlanningSceneTest, extractPoseStampedFromCollisionObject);
            bool extractPoseStampedFromCollisionObject(const moveit_msgs::CollisionObject &co,
            		geometry_msgs::PoseStamped & pose) const;

            FRIEND_TEST(stateCreatorFromPlanningSceneTest, distanceBetweenTwoPoses);
            std::pair<double, double> distanceBetweenTwoPoses(const geometry_msgs::PoseStamped & posePS,
                    const geometry_msgs::PoseStamped & poseState);

		private:

            moveit_msgs::PlanningScene planning_scene_;
            ros::ServiceClient srvPlanningScene_;
            double object_match_distance_;
            double object_z_match_distance_;
            tf::TransformListener tf_;

            void initializePlanningScene();
            void setPlanningScene(const moveit_msgs::PlanningScene& scene);

            FRIEND_TEST(stateCreatorFromPlanningSceneTest, checkIfTableInState);
            bool checkIfTableInState(const SymbolicState& state, const std::string& tableName);

            FRIEND_TEST(stateCreatorFromPlanningSceneTest, isMatch);
            inline bool isMatch(const std::pair<double, double>& distance, double match_distance,
            		double z_match_distance) {
            	return (distance.first <= match_distance && distance.second <= z_match_distance);
            }

            FRIEND_TEST(stateCreatorFromPlanningSceneTest, addObjectToState);
            void addObjectToState(SymbolicState & state, const moveit_msgs::CollisionObject& co,
            		const std::string& objectType);

	};

};

#endif

