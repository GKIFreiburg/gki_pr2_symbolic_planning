#ifndef GOAL_CREATOR_MOVE_OBJECTS_TO_TABLE_H
#define GOAL_CREATOR_MOVE_OBJECTS_TO_TABLE_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_state_creators
{

    class GoalCreatorMoveObjectsToTable : public continual_planning_executive::GoalCreator
    {
    private:
        public:
            GoalCreatorMoveObjectsToTable();
            ~GoalCreatorMoveObjectsToTable();

            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif

