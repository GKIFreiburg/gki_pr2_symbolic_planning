#ifndef GOAL_CREATOR_DEFINE_GOAL_TIDYUP_EXPERIMENT_H
#define GOAL_CREATOR_DEFINE_GOAL_TIDYUP_EXPERIMENT_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_state_creators
{

    class GoalCreatorDefineGoalTidyupExperiment : public continual_planning_executive::GoalCreator
    {
    private:
        public:
            GoalCreatorDefineGoalTidyupExperiment();
            ~GoalCreatorDefineGoalTidyupExperiment();

            virtual void initialize(const std::deque<std::string> & arguments);
            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif // GOAL_CREATOR_DEFINE_GOAL_TIDYUP_EXPERIMENT_H

