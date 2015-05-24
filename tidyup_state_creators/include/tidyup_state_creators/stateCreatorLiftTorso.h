#ifndef STATE_CREATOR_LIFT_TORSO_H
#define STATE_CREATOR_LIFT_TORSO_H

#include "continual_planning_executive/stateCreator.h"
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>

namespace tidyup_state_creators
{

	// StateCreator with only object to write torso position (= joint value) into symbolic state
    class StateCreatorLiftTorso : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorLiftTorso();
            ~StateCreatorLiftTorso();

            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

        private:
            moveit::planning_interface::MoveGroup* torso_group_;

            std::string torso_position_;
            std::string predicate_torso_lifted_;
            tf::TransformListener tf_;

            // LiftTorso-Parameters
            double vdist_head_to_table_;
            double vdist_threshold_;

            bool verifyHeadTableDistance(SymbolicState & state, const std::string& table);
            bool verifyTorsoMaximum();

    };

};

#endif

