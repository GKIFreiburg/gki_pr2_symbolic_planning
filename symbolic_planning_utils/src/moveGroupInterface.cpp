#include "symbolic_planning_utils/moveGroupInterface.h"

namespace symbolic_planning_utils
{
//MoveGroupInterface* MoveGroupInterface::instance = new MoveGroupInterface();
MoveGroupInterface* MoveGroupInterface::instance_ = NULL;

MoveGroupInterface::MoveGroupInterface()
{
	right_arm_group_ = new moveit::planning_interface::MoveGroup("right_arm");
	left_arm_group_ = new moveit::planning_interface::MoveGroup("left_arm");
	arms_group_ = new moveit::planning_interface::MoveGroup("arms");
	head_group_ = new moveit::planning_interface::MoveGroup("head");
	torso_group_ = new moveit::planning_interface::MoveGroup("torso");
}

//MoveGroupInterface::MoveGroupInterface(const MoveGroupInterface* mgi)
//{
//	MoveGroupInterface::instance_ = mgi->instance_;
//}

MoveGroupInterface* MoveGroupInterface::operator= (const MoveGroupInterface* mgi)
{
	if (this != mgi)
	{
		MoveGroupInterface::instance_ = mgi->instance_;
	}
	return this;
}

MoveGroupInterface::~MoveGroupInterface()
{
	delete right_arm_group_;
	delete left_arm_group_;
	delete arms_group_;
	delete head_group_;
	delete torso_group_;
}

MoveGroupInterface* MoveGroupInterface::getInstance()
{
	if (MoveGroupInterface::instance_ == NULL)
	{
		MoveGroupInterface::instance_ = new MoveGroupInterface();
	}
	return MoveGroupInterface::instance_;
}

moveit::planning_interface::MoveGroup* MoveGroupInterface::getRightArmGroup()
{
	return right_arm_group_;
}

moveit::planning_interface::MoveGroup* MoveGroupInterface::getLeftArmGroup()
{
	return left_arm_group_;
}

moveit::planning_interface::MoveGroup* MoveGroupInterface::getArmsGroup()
{
	return arms_group_;
}

moveit::planning_interface::MoveGroup* MoveGroupInterface::getHeadGroup()
{
	return head_group_;
}

moveit::planning_interface::MoveGroup* MoveGroupInterface::getTorsoGroup()
{
	return torso_group_;
}

moveit::planning_interface::MoveItErrorCode MoveGroupInterface::planExecuteVerify(
		moveit::planning_interface::MoveGroup* group,
		const std::string& rosout_name)
{
	moveit::planning_interface::MoveItErrorCode error_code;
	moveit::planning_interface::MoveGroup::Plan plan;
	ROS_DEBUG_STREAM(rosout_name<<": planning motion...");
	error_code = group->plan(plan);
	if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_STREAM(rosout_name<<": motion planning failed. error code: "<<error_code);
		return error_code;
	}

	// planning was successful
	ROS_DEBUG_STREAM(rosout_name<<": executing motion...");
	error_code = group->execute(plan);

	// FIXME: gazebo controller execution does the motion as intended but reports abort in the end
	// manually check joint positions
	double tolerance = 0.05; // 3 degrees
	if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ros::Duration(1).sleep();
		ROS_WARN_STREAM(rosout_name<<": motion execution failed. error code: "<<error_code);
		const trajectory_msgs::JointTrajectory& trajectory = plan.trajectory_.joint_trajectory;
		const trajectory_msgs::JointTrajectoryPoint& target = trajectory.points.back();
		const std::vector<double>& joint_values = group->getCurrentJointValues();
		for (size_t i = 0; i < trajectory.joint_names.size(); i++)
		{
			double diff = fabs(joint_values[i] - target.positions[i]);
			if(diff > tolerance)
			{
				ROS_ERROR_STREAM(rosout_name<<": joint "<<trajectory.joint_names[i]<<": diff "<<diff<<" greater than goal tolerance: "<<tolerance);
				return error_code;
			}
		}
		return moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	return error_code;
}

};
