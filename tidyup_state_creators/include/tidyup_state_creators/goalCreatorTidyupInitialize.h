#ifndef GOAL_CREATOR_TIDYUP_INITIALIZE_H
#define GOAL_CREATOR_TIDYUP_INITIALIZE_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_state_creators
{

    class GoalCreatorTidyupInitialize : public continual_planning_executive::GoalCreator
    {
    private:
        public:
            GoalCreatorTidyupInitialize();
            ~GoalCreatorTidyupInitialize();

            virtual void initialize(const std::deque<std::string> & arguments);
            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif // GOAL_CREATOR_TIDYUP_INITIALIZE_H

