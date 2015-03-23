#ifndef ACTION_EXECUTOR_ARM_TO_SIDE_INTERFACE_H
#define ACTION_EXECUTOR_ARM_TO_SIDE_INTERFACE_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <moveit/move_group_interface/move_group.h>
#include <tidyup_msgs/ArmToSideAction.h>

namespace object_manipulation_actions
{
    class ActionExecutorArmToSide : public continual_planning_executive::ActionExecutorInterface
    {
        public:

			virtual void initialize(const std::deque<std::string> & arguments);

			virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

			virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

			virtual void cancelAction();

        private:

			std::string actionName_;
            std::string armStatePredicateName_;     // the arm state predicate name
            std::string armAtSideConstantName_;     // the arm at side position constant name

    		moveit::planning_interface::MoveGroup* right_arm_group_;
    		moveit::planning_interface::MoveGroup* left_arm_group_;

    		moveit::planning_interface::MoveItErrorCode armToSide(moveit::planning_interface::MoveGroup* group);

    };

};

#endif

