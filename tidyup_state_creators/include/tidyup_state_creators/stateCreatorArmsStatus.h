#ifndef STATE_CREATOR_ARMS_STATUS_H
#define STATE_CREATOR_ARMS_STATUS_H

#include "continual_planning_executive/stateCreator.h"
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

namespace tidyup_state_creators
{

    /// This state creator adds the arm-at-side predicate for both arms
    class StateCreatorArmsStatus : public continual_planning_executive::StateCreator
    {
        public:
			static const std::string ARM_TO_SIDE;
			static const std::string ARM_TO_FRONT;

            StateCreatorArmsStatus();
            ~StateCreatorArmsStatus();

            /// Initialize the state creator parameters.
            /**
             * args: service_name predicate_name predicate_value
             *
             * The service name of the arms-at-side service.
             */
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

        protected:
            string predicate_name_;
            string predicate_at_side_;
            string predicate_at_front_;
            string predicate_unkown_;

            moveit::planning_interface::MoveGroup* right_arm_;
            moveit::planning_interface::MoveGroup* left_arm_;

            // Check if arm group satisfies a named target position which is defined in the pr2.srdf
            bool checkIfArmInTargetPosition(moveit::planning_interface::MoveGroup* group, const std::string& target);

            // Check all named target position for arm group and update arm-state in symbolic state
            void setArmStatusInSymbolicState(SymbolicState& state, moveit::planning_interface::MoveGroup* group);
    };

};

#endif

