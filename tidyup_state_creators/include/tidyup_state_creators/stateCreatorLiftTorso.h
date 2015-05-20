#ifndef STATE_CREATOR_LIFT_TORSO_H
#define STATE_CREATOR_LIFT_TORSO_H

#include "continual_planning_executive/stateCreator.h"
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

namespace tidyup_state_creators
{
    class StateCreatorLiftTorso : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorLiftTorso();
            ~StateCreatorLiftTorso();

            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

        private:
            moveit::planning_interface::MoveGroup* torso_group_;

    };

};

#endif

