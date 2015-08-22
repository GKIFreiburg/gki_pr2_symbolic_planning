#ifndef STATE_CREATOR_LIFT_TORSO_GROUNDING_H
#define STATE_CREATOR_LIFT_TORSO_GROUNDING_H

#include "continual_planning_executive/stateCreator.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <moveit/move_group_interface/move_group.h>

namespace tidyup_state_creators
{

	// StateCreator with only object to write torso position (= joint value) into symbolic state
    class StateCreatorLiftTorsoGrounding : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorLiftTorsoGrounding();
            ~StateCreatorLiftTorsoGrounding();

            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

        private:
//            moveit::planning_interface::MoveGroup* torso_group_;

            tf::TransformListener tf_;

            std::string current_torso_height_;
            std::string sampled_torso_height_;
            std::string predicate_torso_lifted_;

            // LiftTorso-Parameters
            double vdist_threshold_;
    };

};

#endif // STATE_CREATOR_LIFT_TORSO_GROUNDING_H
