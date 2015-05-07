#include "object_manipulation_actions/actionExecutorArmToFront.h"
#include <pluginlib/class_list_macros.h>
#include <tidyup_utils/stringutil.h>
#include <symbolic_planning_utils/moveGroupInterface.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// MoveIt!
#include <moveit/move_group_interface/move_group.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorArmToFront, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
    const std::string ActionExecutorArmToFront::ARM_TO_FRONT  = "_to_front";

	void ActionExecutorArmToFront::initialize(const std::deque<std::string> & arguments)
	{
		actionName_ = arguments[0]; 	// arm-to-front
	}

	bool ActionExecutorArmToFront::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
	    return a.name == actionName_;
	}

	bool ActionExecutorArmToFront::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
//		DEFAULT_RIGHT_ARM_INSPECT_POSE = PoseStamped(Header(frame_id='/head_mount_kinect_rgb_link'),Pose(Point(0.48, -0.2, 0.0), Quaternion(-0.037, -0.031, 0.609, 0.792)))
//		DEFAULT_LEFT_ARM_INSPECT_POSE = PoseStamped(Header(frame_id='/head_mount_kinect_rgb_link'),Pose(Point(0.48, 0.2, 0.0), Quaternion(-0.073, -0.047, -0.669, 0.738)))

		ROS_ASSERT(a.parameters.size() == 1);
		moveit::planning_interface::MoveItErrorCode error_code;
		moveit::planning_interface::MoveGroup* arm_group_;

		if (StringUtil::startsWith(a.parameters[0], "left_"))
		{
			arm_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
		}
		else if (StringUtil::startsWith(a.parameters[0], "right_"))
		{
			arm_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
		}
		else
		{
			ROS_ERROR("ActionExecutorArmToFront::%s: No arm group could be specified.", __func__);
			return false;
		}
		error_code = armToFront(arm_group_);
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	void ActionExecutorArmToFront::cancelAction()
	{

	}

	moveit::planning_interface::MoveItErrorCode ActionExecutorArmToFront::armToFront(moveit::planning_interface::MoveGroup* group)
	{
		std::string target = group->getName() + ARM_TO_FRONT;
		if (!group->setNamedTarget(target))
		{
			ROS_ERROR("ActionExecutorArmToFront::%s: Could not find named target: %s", __func__, target.c_str());
			return  moveit::planning_interface::MoveItErrorCode::FAILURE;
		}

		moveit::planning_interface::MoveItErrorCode error_code;

		// Call the planner to compute a plan.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		moveit::planning_interface::MoveGroup::Plan my_plan;
		ROS_DEBUG("ActionExecutorArmToFront::%s: planning arm motion...", __func__);

        error_code = group->plan(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ActionExecutorArmToFront::%s: Ups, something with arm motion planning went wrong.", __func__);
			return error_code;
		}

		// planning was successful
		ROS_DEBUG("ActionExecutorArmToFront::%s: executing arm motion...", __func__);
		error_code = group->execute(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ActionExecutorArmToFront::%s: Ups, something with arm motion execution went wrong.", __func__);
			return error_code;
		}

		return error_code;
	}

};

