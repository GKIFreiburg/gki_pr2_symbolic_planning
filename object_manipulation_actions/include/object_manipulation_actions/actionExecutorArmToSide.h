#ifndef ACTION_EXECUTOR_ARM_TO_SIDE_H
#define ACTION_EXECUTOR_ARM_TO_SIDE_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <moveit/move_group_interface/move_group.h>

namespace object_manipulation_actions
{
    class ActionExecutorArmToSide : public continual_planning_executive::ActionExecutorInterface
    {
        public:
			static const std::string ARM_TO_SIDE;

			virtual void initialize(const std::deque<std::string> & arguments);

			virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

			virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

			virtual void cancelAction();

        private:

			std::string actionName_;
			std::string named_target_right_arm_to_side_;
			std::string named_target_left_arm_to_side_;

			moveit::planning_interface::MoveGroup* left_arm_;
			moveit::planning_interface::MoveGroup* right_arm_;

    		moveit::planning_interface::MoveItErrorCode executeArmToSide(
    				moveit::planning_interface::MoveGroup* group, const std::string& target);

    };

};

#endif

