#include "object_manipulation_actions/actionExecutorArmToSide.h"
#include <pluginlib/class_list_macros.h>
#include "tidyup_utils/planning_scene_interface.h"
#include <tidyup/arms_at_side.h>
#include <moveit/move_group_interface/move_group.h>

//PLUGINLIB_DECLARE_CLASS(object_manipulation_actions, action_executor_arm_to_side,
//        object_manipulation_actions::ActionExecutorArmToSide,
//        continual_planning_executive::ActionExecutorInterface)
PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorArmToSide, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{

	void ActionExecutorArmToSide::initialize(const std::deque<std::string> & arguments)
	{
        _armStatePredicateName = "arm-state";
        _armAtSideConstantName = "arm_at_side";

//		right_arm_group_ = new moveit::planning_interface::MoveGroup("right_arm");
//		left_arm_group_ = new moveit::planning_interface::MoveGroup("left_arm");
//		moveit::planning_interface::MoveGroup r("right_arm");
//		moveit::planning_interface::MoveGroup l("left_arm");
//		right_arm_group_ = &r;
//		left_arm_group_  = &l;

        // TODO: ????
//        if(arguments.size() >= 3) {     // 3rd param: arm-state predicate name
//            _armStatePredicateName = arguments[2];
//        }
//        if(arguments.size() >= 4) {     // 4th param: arm_at_side constant name
//            _armAtSideConstantName = arguments[3];
//        }
	}

	bool ActionExecutorArmToSide::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
//		ROS_ASSERT(a.parameters.size() == 1);
//		if(a.parameters[0] == left_arm_group_->getName()) {
//			std::vector<double> jointValues;
//			if (!tidyup::armsAtSide::loadJointValues(left_arm_group_->getName(), jointValues))
//			{
//				ROS_ERROR("ActionExecutorArmToSide::%s: Could not load joint values for left arm"
//						"from param server", __func__);
//				return false;
//			}
//
//			left_arm_group_->setJointValueTarget(jointValues);
//
//			// Call the planner to compute a plan.
//			// Note that we are just planning, not asking move_group
//			// to actually move the robot.
//			moveit::planning_interface::MoveGroup::Plan my_plan;
//			bool result = left_arm_group_->plan(my_plan);
//			return result;
//
//		} else if(a.parameters[0] == right_arm_group_->getName()) {
//			std::vector<double> jointValues;
//			if (!tidyup::armsAtSide::loadJointValues(right_arm_group_->getName(), jointValues))
//			{
//				ROS_ERROR("ActionExecutorArmToSide::%s: Could not load joint values for right arm"
//						"from param server", __func__);
//				return false;
//			}
//
//			right_arm_group_->setJointValueTarget(jointValues);
//
//			// Call the planner to compute a plan.
//			// Note that we are just planning, not asking move_group
//			// to actually move the robot.
//			moveit::planning_interface::MoveGroup::Plan my_plan;
//			bool result = right_arm_group_->plan(my_plan);
//			return result;
//		}
//		ROS_ERROR("ActionExecutorArmToSide::%s: Something went wrong. Got wrong durativeAction!", __func__);
		return false;
	}

	bool ActionExecutorArmToSide::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
		ROS_WARN("EVERYTHING IS WORKING!");
		return true;
	}

	void ActionExecutorArmToSide::cancelAction()
	{

	}

};

