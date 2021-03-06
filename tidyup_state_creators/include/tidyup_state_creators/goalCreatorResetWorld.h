#ifndef GOAL_CREATOR_RESET_WORLD_H
#define GOAL_CREATOR_RESET_WORLD_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_state_creators
{

    class GoalCreatorResetWorld : public continual_planning_executive::GoalCreator
    {
    private:
    	static bool worldResetPerformed;
        public:
            GoalCreatorResetWorld();
            ~GoalCreatorResetWorld();

            virtual void initialize(const std::deque<std::string> & arguments);
            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif
