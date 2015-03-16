#ifndef STATE_CREATOR_FROM_PLANNING_SCENE_H
#define STATE_CREATOR_FROM_PLANNING_SCENE_H

#include "continual_planning_executive/stateCreator.h"
#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/PoseStamped.h>
#include <set>

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
            bool extractPoseStampedFromSymbolicState(const SymbolicState & state, const string & object,
                    geometry_msgs::PoseStamped & pose) const;

            // the same as in ork_to_planning_scene
            bool extractPoseStampedFromCollisionObject(const moveit_msgs::CollisionObject &co,
            		geometry_msgs::PoseStamped & pose) const;

            std::pair<double, double> distanceBetweenTwoPoses(const geometry_msgs::PoseStamped & posePS,
                    const geometry_msgs::PoseStamped & poseState);

		private:

            moveit_msgs::PlanningScene planning_scene_;
            double object_match_distance_;
            double object_z_match_distance_;

            void initializePlanningScene();
            void setInitialScene(const moveit_msgs::PlanningScene& scene);

            bool checkIfTableInState(const SymbolicState& state, const std::string& table);

            inline bool isMatch(const std::pair<double, double>& distance, double match_distance,
            		double z_match_distance) {
            	return (distance.first <= match_distance && distance.second <= z_match_distance);
            }

            void addObjectToState(SymbolicState & state, const moveit_msgs::CollisionObject& co,
            		const std::string& objectType);

	};

};

#endif

