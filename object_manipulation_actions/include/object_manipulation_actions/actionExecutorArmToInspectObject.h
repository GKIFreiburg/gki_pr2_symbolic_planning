#ifndef ACTION_EXECUTOR_ARM_TO_INSPECT_H
#define ACTION_EXECUTOR_ARM_TO_INSPECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/ArmToInspectObjectAction.h>

namespace object_manipulation_actions
{

    class ActionExecutorArmToInspectObject : public ActionExecutorActionlib<tidyup_msgs::ArmToInspectObjectAction,
    tidyup_msgs::ArmToInspectObjectGoal, tidyup_msgs::ArmToInspectObjectResult>
    {
        public:
            /**
             * Initialize the ArmToInspectObject action using the following parameters:
             * action_plan_name action_server_name [armStatePredicate [armAtInspectObjectConstant]]
             *
             * armStatePredicate and armAtInspectObjectConstant give the predicate that is set to the at InspectObject
             * constant when the action succeeds.
             */
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillGoal(tidyup_msgs::ArmToInspectObjectGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState, const tidyup_msgs::ArmToInspectObjectResult & result,
                    const DurativeAction & a, SymbolicState & current);

        private:
            std::string _armStatePredicateName;     // the arm state predicate name
            std::string _armAtInspectObjectConstantName;     // the arm at InspectObject position constant name

    };

};

#endif

