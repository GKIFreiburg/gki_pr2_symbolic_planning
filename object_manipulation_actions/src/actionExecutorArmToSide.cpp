#include "object_manipulation_actions/actionExecutorArmToSide.h"
#include <pluginlib/class_list_macros.h>
#include <tidyup_utils/stringutil.h>
#include <symbolic_planning_utils/moveGroupInterface.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// MoveIt!
#include <moveit/move_group_interface/move_group.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorArmToSide, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
	void ActionExecutorArmToSide::initialize(const std::deque<std::string> & arguments)
	{
		ROS_ASSERT(arguments.size() == 3);

		actionName_ 					= arguments[0]; 	// arm-to-side
		named_target_right_arm_to_side_ = arguments[1];		// right_arm_to_side
		named_target_left_arm_to_side_  = arguments[2];		// left_arm_to_side

		left_arm_ = new moveit::planning_interface::MoveGroup("left_arm");
		right_arm_ = new moveit::planning_interface::MoveGroup("right_arm");
	}

	bool ActionExecutorArmToSide::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
	    return a.name == actionName_;
	}

	bool ActionExecutorArmToSide::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
		ROS_ASSERT(a.parameters.size() == 1);
		moveit::planning_interface::MoveGroup* arm_group;
		moveit::planning_interface::MoveItErrorCode error_code;
		std::string target;

		if (StringUtil::startsWith(a.parameters[0], "left_"))
		{
//			arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
			arm_group = left_arm_;
			target = named_target_left_arm_to_side_;
		}
		else if (StringUtil::startsWith(a.parameters[0], "right_"))
		{
//			arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
			arm_group = right_arm_;
			target = named_target_right_arm_to_side_;
		}
		else
		{
			ROS_ERROR("ActionExecutorArmToSide::%s: No arm group could be specified.", __func__);
			return false;
		}
		error_code = executeArmToSide(arm_group, target);
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	void ActionExecutorArmToSide::cancelAction()
	{

	}

	moveit::planning_interface::MoveItErrorCode ActionExecutorArmToSide::executeArmToSide(
			moveit::planning_interface::MoveGroup* group, const std::string& target)
	{
		if (!group->setNamedTarget(target))
		{
			ROS_ERROR("ActionExecutorArmToSide::%s: Could not find named target: %s", __func__, target.c_str());
			return  moveit::planning_interface::MoveItErrorCode::FAILURE;
		}

		moveit::planning_interface::MoveItErrorCode error_code;

		// Call the planner to compute a plan.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		moveit::planning_interface::MoveGroup::Plan my_plan;
		ROS_DEBUG("ActionExecutorArmToSide::%s: planning arm motion...", __func__);

        error_code = group->plan(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ActionExecutorArmToSide::%s: Ups, something with arm motion planning went wrong.", __func__);
			return error_code;
		}

		// planning was successful
		ROS_DEBUG("ActionExecutorArmToSide::%s: executing arm motion...", __func__);
		error_code = group->execute(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ActionExecutorArmToSide::%s: Ups, something with arm motion execution went wrong.", __func__);
			return error_code;
		}

		return error_code;
	}

};

