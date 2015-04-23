#include "object_manipulation_actions/actionExecutorArmToSide.h"
#include <pluginlib/class_list_macros.h>
#include <tidyup_msgs/ArmsAtSide.h>
#include <tidyup_utils/arms_at_side.h>
#include <tidyup_utils/stringutil.h>
#include <symbolic_planning_utils/moveGroupInterface.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// MoveIt!
#include <moveit/move_group_interface/move_group.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/planning_pipeline/planning_pipeline.h>
//#include <moveit/planning_interface/planning_interface.h>
//#include <moveit/kinematic_constraints/utils.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/PlanningScene.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorArmToSide, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{

	void ActionExecutorArmToSide::initialize(const std::deque<std::string> & arguments)
	{
        armStatePredicateName_ = "arm-state";
        armAtSideConstantName_ = "arm_at_side";

	    actionName_ = arguments.at(0);
	    ROS_INFO("Initializing ActionExecutor for action %s...", actionName_.c_str());
        if(arguments.size() >= 3) {     // 3rd param: arm-state predicate name
            armStatePredicateName_ = arguments[2];
        }
        if(arguments.size() >= 4) {     // 4th param: arm_at_side constant name
            armAtSideConstantName_ = arguments[3];
        }
	}

	bool ActionExecutorArmToSide::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
	    return a.name == actionName_;
	}

	bool ActionExecutorArmToSide::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
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
			ROS_ERROR("ActionExecutorArmToSide::%s: No arm group could be specified.", __func__);
			return false;
		}
		error_code = armToSide(arm_group_);
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	void ActionExecutorArmToSide::cancelAction()
	{

	}

	moveit::planning_interface::MoveItErrorCode ActionExecutorArmToSide::armToSide(moveit::planning_interface::MoveGroup* group)
	{
		// get the joint values for arm side position from param server
        std::vector<double> jointValues;
        if (!tidyup::ArmsAtSide::loadJointValues(group->getName(), jointValues))
        {
            ROS_ERROR("ActionExecutorArmToSide::%s: Could not load joint values for arm "
                    "from param server", __func__);
            return moveit::planning_interface::MoveItErrorCode::FAILURE;
        }
		ROS_ASSERT(jointValues.size() > 0);

        ROS_INFO("ActionExecutorArmToSide::%s: setting joint values", __func__);
        group->setJointValueTarget(jointValues);

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

		// error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS
		return error_code;
	}

};

