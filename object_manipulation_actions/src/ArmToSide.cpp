#include "object_manipulation_actions/ArmToSide.h"

#include <pluginlib/class_list_macros.h>
#include <tidyup_utils/stringutil.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <symbolic_planning_utils/moveGroupInterface.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ArmToSide, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
	void ArmToSide::initialize(const std::deque<std::string> & arguments)
	{
		ROS_ASSERT(arguments.size() == 3);

		actionName_ 					= arguments[0]; 	// arm-to-side
		named_target_right_arm_to_side_ = arguments[1];		// right_arm_to_side
		named_target_left_arm_to_side_  = arguments[2];		// left_arm_to_side

		left_arm_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
		right_arm_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
	}

	bool ArmToSide::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
		return a.name == actionName_;
	}

	bool ArmToSide::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
		ROS_ASSERT(a.parameters.size() == 1);
		moveit::planning_interface::MoveGroup* arm_group;
		moveit::planning_interface::MoveItErrorCode error_code;
		std::string target;

		if (StringUtil::startsWith(a.parameters[0], "left_"))
		{
			arm_group = left_arm_;
			target = named_target_left_arm_to_side_;
		}
		else if (StringUtil::startsWith(a.parameters[0], "right_"))
		{
			arm_group = right_arm_;
			target = named_target_right_arm_to_side_;
		}
		else
		{
			ROS_ERROR_STREAM(actionName_<<": arm group lookup failed. expected right_ or left_, got "<<a.parameters[0]);
			return false;
		}
		error_code = executeArmToSide(arm_group, target);
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	void ArmToSide::cancelAction()
	{

	}

	moveit::planning_interface::MoveItErrorCode ArmToSide::executeArmToSide(
			moveit::planning_interface::MoveGroup* group, const std::string& target)
	{
		if (!group->setNamedTarget(target))
		{
			ROS_ERROR("ActionExecutorArmToSide::%s: named target not found: %s", __func__, target.c_str());
			return  moveit::planning_interface::MoveItErrorCode::FAILURE;
		}

		return symbolic_planning_utils::MoveGroupInterface::getInstance()->planExecuteVerify(group, actionName_);
	}

};

