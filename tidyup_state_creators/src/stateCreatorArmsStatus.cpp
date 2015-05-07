#include "tidyup_state_creators/stateCreatorArmsStatus.h"
#include <pluginlib/class_list_macros.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorArmsStatus, continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{
    const std::string StateCreatorArmsStatus::ARM_TO_SIDE  = "_to_side";
    const std::string StateCreatorArmsStatus::ARM_TO_FRONT = "_to_front";

    StateCreatorArmsStatus::StateCreatorArmsStatus()
    {
    }

    StateCreatorArmsStatus::~StateCreatorArmsStatus()
    {
    }

    void StateCreatorArmsStatus::initialize(const std::deque<std::string> & arguments)
    {
        ROS_ASSERT(arguments.size() == 4);
        predicate_name_     = arguments[0];		// arm-state
        predicate_at_side_  = arguments[1];		// arm_at_side
        predicate_at_front_ = arguments[2];		// arm_at_front
        predicate_unkown_   = arguments[3];		// arm_unkown

        right_arm_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
        left_arm_  = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
    }

    bool StateCreatorArmsStatus::fillState(SymbolicState & state)
    {
    	setArmStatusInSymbolicState(state, right_arm_);
    	setArmStatusInSymbolicState(state, left_arm_);
        return true;
    }

    bool StateCreatorArmsStatus::checkIfArmInTargetPosition(
    		moveit::planning_interface::MoveGroup* group, const std::string& target)
    {
		std::vector<double> current_joint_values = group->getCurrentJointValues();
		std::vector<std::string> current_joint_names = group->getJoints();

		if (!group->setNamedTarget(target))
		{
			ROS_ERROR("StateCreatorArmsStatus::%s: Could not find named target: %s", __func__, target.c_str());
			return false;
		}

		std::vector<double> target_joint_values;
		const robot_state::RobotState& rs = group->getJointValueTarget();
		const robot_state::JointModelGroup* joint_model_group = rs.getJointModelGroup(group->getName());
		const std::vector<std::string> &target_joint_names = joint_model_group->getJointModelNames();
		rs.copyJointGroupPositions(joint_model_group, target_joint_values);

  		// Error needed since joints never reach exactly the given values.
  		double error = 0.02;
  		ROS_ASSERT(current_joint_values.size() == target_joint_values.size());
  		ROS_ASSERT(current_joint_names.size() == target_joint_names.size());

  		for (int i = 0; i < current_joint_names.size(); i++)
  		{
  			// taking care of joints that can rotate 360 degrees (continuous type)
  			// affected joints are: *_fore_arm_roll_joint and *_wrist_roll_joint
  			// joint values should be in the range of [-PI, +PI]
  			while (fabs(current_joint_values[i]) > M_PI)
  			{
  				ROS_INFO("StateCreatorArmsStatus::%s: %s: %lf > %lf", __func__,
  						current_joint_names[i].c_str(), current_joint_values[i], 2*M_PI);
  				if (current_joint_values[i] > 0) // positive
  					current_joint_values[i] = current_joint_values[i] - 2*M_PI;
  				else // negative
  					current_joint_values[i] = current_joint_values[i] + 2*M_PI;
  			}

  			ROS_INFO("StateCreatorArmsStatus::%s: current %s - target %s : %lf - %lf",__func__ ,
  					 current_joint_names[i].c_str(), target_joint_names[i].c_str(),
  					 current_joint_values[i], target_joint_values[i]);
  			if (std::fabs((double)current_joint_values[i] - (double)target_joint_values[i]) > error)
  				return false;
  		}
    	return true;
    }

    void StateCreatorArmsStatus::setArmStatusInSymbolicState(
    		SymbolicState& state, moveit::planning_interface::MoveGroup* group)
    {
    	std::string target = group->getName() + ARM_TO_SIDE;
    	if (checkIfArmInTargetPosition(group, target))
    	{
    		state.setObjectFluent(predicate_name_, group->getName(), predicate_at_side_);
    		return;
    	}

    	target = group->getName() + ARM_TO_FRONT;
    	if (checkIfArmInTargetPosition(group, target))
    	{
    		state.setObjectFluent(predicate_name_, group->getName(), predicate_at_front_);
    		return;
    	}

    	state.setObjectFluent(predicate_name_, group->getName(), predicate_unkown_);
    }
};

