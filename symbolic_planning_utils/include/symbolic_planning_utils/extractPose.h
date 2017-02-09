#ifndef EXTRACT_POSE_H_
#define EXTRACT_POSE_H_

#include <continual_planning_executive/symbolicState.h>
#include <geometry_msgs/PoseStamped.h>

namespace symbolic_planning_utils
{
	bool extractPoseStampedFromSymbolicState(const SymbolicState & state, const std::string & object,
			geometry_msgs::PoseStamped & pose);

	bool comparePoses(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, double tolerance = 0);

	bool comparePoseStampeds(const geometry_msgs::PoseStamped& p1,
			const geometry_msgs::PoseStamped& p2, double tolerance = 0);

};

#endif
