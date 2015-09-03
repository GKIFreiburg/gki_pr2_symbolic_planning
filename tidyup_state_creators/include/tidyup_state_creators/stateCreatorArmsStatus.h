#ifndef STATE_CREATOR_ARMS_STATUS_H
#define STATE_CREATOR_ARMS_STATUS_H

#include "continual_planning_executive/stateCreator.h"
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>

namespace tidyup_state_creators
{

    /// This state creator adds the arm-at-side predicate for both arms
    class StateCreatorArmsStatus : public continual_planning_executive::StateCreator
    {
        public:
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
            string named_target_right_arm_to_side_;
            string named_target_left_arm_to_side_;
            string named_target_right_arm_to_front_;
            string named_target_left_arm_to_front_;

            moveit::planning_interface::MoveGroup* right_arm_;
            moveit::planning_interface::MoveGroup* left_arm_;

    	    tf::TransformListener tf_;

            // Check all named target position for arm group and update arm-state in symbolic state
            void setArmStatusInSymbolicState(SymbolicState& state, moveit::planning_interface::MoveGroup* group);

            // Check if arm group satisfies a named target position which is defined in the pr2.srdf
            bool checkIfArmInTargetPosition(moveit::planning_interface::MoveGroup* group, const std::string& target);

            void normalizeJointValue(double& jointValue);

    };

};

#endif

