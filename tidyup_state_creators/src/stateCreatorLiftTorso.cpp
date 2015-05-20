#include "tidyup_state_creators/stateCreatorLiftTorso.h"
#include <pluginlib/class_list_macros.h>
#include <symbolic_planning_utils/moveGroupInterface.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorLiftTorso, continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{
    StateCreatorLiftTorso::StateCreatorLiftTorso()
    {
    	torso_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getTorsoGroup();
    }

    StateCreatorLiftTorso::~StateCreatorLiftTorso()
    {
    }

    void StateCreatorLiftTorso::initialize(const std::deque<std::string> & arguments)
    {

    }

    bool StateCreatorLiftTorso::fillState(SymbolicState & state)
    {
        return true;
    }
};

