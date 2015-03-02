#include "object_manipulation_actions/actionExecutorArmToSideInterface.h"
#include <pluginlib/class_list_macros.h>
//#include "tidyup_utils/planning_scene_interface.h"
#include <tidyup_msgs/ArmsAtSide.h>
#include <moveit/move_group_interface/move_group.h>
#include <tidyup_utils/arms_at_side.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>



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

		right_arm_group_ = new moveit::planning_interface::MoveGroup("right_arm");
		left_arm_group_ = new moveit::planning_interface::MoveGroup("left_arm");

		// TODO: ????
	    _actionName = arguments.at(0);
	    ROS_INFO("Initializing ActionExecutor for action %s...", _actionName.c_str());
        if(arguments.size() >= 3) {     // 3rd param: arm-state predicate name
            _armStatePredicateName = arguments[2];
        }
        if(arguments.size() >= 4) {     // 4th param: arm_at_side constant name
            _armAtSideConstantName = arguments[3];
        }
	}

	bool ActionExecutorArmToSide::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
	    return a.name == _actionName;
	}

bool ActionExecutorArmToSide::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
{
    ROS_ASSERT(a.parameters.size() == 1);
    if(a.parameters[0] == left_arm_group_->getName()) {
        std::vector<double> jointValues;
        if (!tidyup::ArmsAtSideServiceServer::loadJointValues(left_arm_group_->getName(), jointValues))
        {
            ROS_ERROR("ActionExecutorArmToSide::%s: Could not load joint values for left arm "
                    "from param server", __func__);
            return false;
        }
        ROS_INFO("setting joint values");
        left_arm_group_->setJointValueTarget(jointValues);

        moveit::planning_interface::MoveItErrorCode error_code = left_arm_group_->move();
        return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;

    } else if(a.parameters[0] == right_arm_group_->getName()) {
        std::vector<double> jointValues;
        if (!tidyup::ArmsAtSideServiceServer::loadJointValues(right_arm_group_->getName(), jointValues))
        {
            ROS_ERROR("ActionExecutorArmToSide::%s: Could not load joint values for right arm "
                    "from param server", __func__);
            return false;
        }

        ROS_INFO("setting joint values");
        right_arm_group_->setJointValueTarget(jointValues);
        moveit::planning_interface::MoveItErrorCode error_code = right_arm_group_->move();
        return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }
    return false;
}

	void ActionExecutorArmToSide::cancelAction()
	{

	}

};

