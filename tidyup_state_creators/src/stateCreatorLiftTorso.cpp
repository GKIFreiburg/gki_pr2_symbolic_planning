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
    	ROS_ASSERT(arguments.size() == 1);
    	torso_position_ = arguments[0];			// torso-position
    }

    bool StateCreatorLiftTorso::fillState(SymbolicState & state)
    {
    	// get current torso position
    	std::vector<double> current_joint_values = torso_group_->getCurrentJointValues();
    	ROS_ASSERT(current_joint_values.size() == 1);

    	state.setNumericalFluent(torso_position_, "", current_joint_values[0]);

        return true;
    }
};

