#ifndef EXTRACT_POSE_H_
#define EXTRACT_POSE_H_

#include <continual_planning_executive/symbolicState.h>
#include <geometry_msgs/PoseStamped.h>

namespace symbolic_planning_utils
{
	bool extractPoseStampedFromSymbolicState(const SymbolicState & state, const std::string & object,
        geometry_msgs::PoseStamped & pose);
};

#endif
