#ifndef JOINT_LIMITS_H_
#define JOINT_LIMITS_H_

#include <moveit/move_group_interface/move_group.h>

namespace symbolic_planning_utils
{
	class JointLimits
	{
		public:

		struct Limits
		{
			double min_position;
			double max_position;
		};

		static std::map<std::string, Limits> getAllJointLimits(moveit::planning_interface::MoveGroup* group);

		static Limits getJointLimit(moveit::planning_interface::MoveGroup* group, const std::string& joint);

	};

}; // namespace

#endif
