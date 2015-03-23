#include "symbolic_planning_utils/moveGroupInterface.h"

namespace symbolic_planning_utils
{
MoveGroupInterface* MoveGroupInterface::instance = new MoveGroupInterface();

MoveGroupInterface::MoveGroupInterface()
{
	right_arm_group_ = new moveit::planning_interface::MoveGroup("right_arm");
	left_arm_group_ = new moveit::planning_interface::MoveGroup("left_arm");
    arms_group_ = new moveit::planning_interface::MoveGroup("arms");
    head_group_ = new moveit::planning_interface::MoveGroup("head");
}

MoveGroupInterface::MoveGroupInterface(const MoveGroupInterface* mgi)
{
	MoveGroupInterface::instance = mgi->instance;
}

MoveGroupInterface* MoveGroupInterface::operator= (const MoveGroupInterface* mgi)
{
	if (this != mgi)
	{
		MoveGroupInterface::instance = mgi->instance;
	}
	return this;
}

MoveGroupInterface::~MoveGroupInterface()
{
}

MoveGroupInterface* MoveGroupInterface::getInstance()
{
    return MoveGroupInterface::instance;
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
};
