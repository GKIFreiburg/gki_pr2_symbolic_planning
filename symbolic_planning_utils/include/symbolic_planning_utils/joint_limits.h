#ifndef JOINT_LIMITS_H_
#define JOINT_LIMITS_H_

#include <moveit/move_group_interface/move_group.h>

namespace symbolic_planning_utils
{
	struct limits
	{
		double min_position;
		double max_position;
	};

	class JointLimits
	{
		public:
		static std::map<std::string, limits> getAllJointLimits(moveit::planning_interface::MoveGroup* group);

		static limits getJointLimit(moveit::planning_interface::MoveGroup* group, const std::string& joint);

	};

}; // namespace

#endif
