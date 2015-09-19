#ifndef GOAL_CREATOR_DEFINE_GOAL_GROUNDING_H
#define GOAL_CREATOR_DEFINE_GOAL_GROUNDING_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_state_creators
{

    class GoalCreatorDefineGoalGrounding : public continual_planning_executive::GoalCreator
    {
    private:
        public:
            GoalCreatorDefineGoalGrounding();
            ~GoalCreatorDefineGoalGrounding();

            virtual void initialize(const std::deque<std::string> & arguments);
            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif // GOAL_CREATOR_DEFINE_GOAL_GROUNDING_H

