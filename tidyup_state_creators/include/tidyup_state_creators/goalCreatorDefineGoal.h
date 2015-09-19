#ifndef GOAL_CREATOR_DEFINE_GOAL_H
#define GOAL_CREATOR_DEFINE_GOAL_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_state_creators
{

    class GoalCreatorDefineGoal : public continual_planning_executive::GoalCreator
    {
    private:
        public:
            GoalCreatorDefineGoal();
            ~GoalCreatorDefineGoal();

            virtual void initialize(const std::deque<std::string> & arguments);
            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif // GOAL_CREATOR_DEFINE_GOAL_H

