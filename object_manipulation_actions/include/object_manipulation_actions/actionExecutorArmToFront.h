#ifndef ACTION_EXECUTOR_ARM_TO_FRONT_H
#define ACTION_EXECUTOR_ARM_TO_FRONT_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <moveit/move_group_interface/move_group.h>

namespace object_manipulation_actions
{
    class ActionExecutorArmToFront : public continual_planning_executive::ActionExecutorInterface
    {
        public:
			static const std::string ARM_TO_FRONT;

			virtual void initialize(const std::deque<std::string> & arguments);

			virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

			virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

			virtual void cancelAction();

        private:

			std::string actionName_;

    		moveit::planning_interface::MoveItErrorCode armToFront(moveit::planning_interface::MoveGroup* group);

    };

};

#endif

