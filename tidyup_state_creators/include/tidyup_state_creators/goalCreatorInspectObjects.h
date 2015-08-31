#ifndef GOAL_CREATOR_MOVE_OBJECTS_TO_TABLE_GROUNDING_H
#define GOAL_CREATOR_MOVE_OBJECTS_TO_TABLE_GROUNDING_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_state_creators
{

    class GoalCreatorInspectObjects : public continual_planning_executive::GoalCreator
    {
    private:
        public:
            GoalCreatorInspectObjects();
            ~GoalCreatorInspectObjects();

            virtual void initialize(const std::deque<std::string> & arguments);
            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif
