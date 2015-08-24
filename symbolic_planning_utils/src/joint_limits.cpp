#include "symbolic_planning_utils/joint_limits.h"
#include <moveit/robot_state/robot_state.h>

namespace symbolic_planning_utils
{
	std::map<std::string, JointLimits::Limits> JointLimits::getAllJointLimits(moveit::planning_interface::MoveGroup* group)
	{
		std::map<std::string, Limits> ret;

	    robot_state::RobotStatePtr robot_state = group->getCurrentState();
	    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(group->getName());

	    const std::vector<const moveit::core::JointModel*> joints = joint_model_group->getActiveJointModels();
	    for (size_t i = 0; i < joints.size(); i++)
	    {
	    	const moveit::core::JointModel* joint_model = joints[i];
	    	std::string joint_name = joint_model->getName();
	    	moveit::core::VariableBounds variable_bound = joint_model->getVariableBounds(joint_name);
	    	Limits limit;
	    	limit.min_position = variable_bound.min_position_;
	    	limit.max_position = variable_bound.max_position_;
	    	std::pair<std::string, Limits> joint_limits = std::make_pair(joint_name, limit);
	    	ret.insert(joint_limits);
	    }

	    return ret;
	}

	JointLimits::Limits JointLimits::getJointLimit(moveit::planning_interface::MoveGroup* group, const std::string& joint)
	{
	    robot_state::RobotStatePtr robot_state = group->getCurrentState();
	    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(group->getName());
	    const moveit::core::JointModel* joint_model = joint_model_group->getJointModel(joint);
	    moveit::core::VariableBounds variable_bound = joint_model->getVariableBounds(joint);
	    Limits limit;
		limit.min_position = variable_bound.min_position_;
		limit.max_position = variable_bound.max_position_;
		return limit;
	}


};



