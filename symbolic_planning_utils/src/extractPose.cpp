#include "symbolic_planning_utils/extractPose.h"

#include <ros/ros.h>

namespace symbolic_planning_utils
{
	bool extractPoseStampedFromSymbolicState(const SymbolicState & state, const std::string & object,
		geometry_msgs::PoseStamped & pose)
	{
		// first get xyz, qxyzw from state
		Predicate p;
		p.parameters.push_back(object);

		double posX = 0;
		p.name = "x";
		if(!state.hasNumericalFluent(p, &posX)) {
			ROS_ERROR("%s: object: %s - no x-location in state.", __func__, object.c_str());
			return false;
		}
		double posY = 0;
		p.name = "y";
		if(!state.hasNumericalFluent(p, &posY)) {
			ROS_ERROR("%s: object: %s - no y-location in state.", __func__, object.c_str());
			return false;
		}
		double posZ = 0;
		p.name = "z";
		if(!state.hasNumericalFluent(p, &posZ)) {
			ROS_ERROR("%s: object: %s - no z-location in state.", __func__, object.c_str());
			return false;
		}

		double qx;
		p.name = "qx";
		if(!state.hasNumericalFluent(p, &qx)) {
			ROS_ERROR("%s: object: %s - no qx in state.", __func__, object.c_str());
			return false;
		}
		double qy;
		p.name = "qy";
		if(!state.hasNumericalFluent(p, &qy)) {
			ROS_ERROR("%s: object: %s - no qy in state.", __func__, object.c_str());
			return false;
		}
		double qz;
		p.name = "qz";
		if(!state.hasNumericalFluent(p, &qz)) {
			ROS_ERROR("%s: object: %s - no qz in state.", __func__, object.c_str());
			return false;
		}
		double qw;
		p.name = "qw";
		if(!state.hasNumericalFluent(p, &qw)) {
			ROS_ERROR("%s: object: %s - no qw in state.", __func__, object.c_str());
			return false;
		}

		double timestamp;
		p.name = "timestamp";
		if(!state.hasNumericalFluent(p, &timestamp)) {
			ROS_ERROR("%s: object: %s - no timestamp in state.", __func__, object.c_str());
			return false;
		}

		string frameid;
		p.name = "frame-id";
		if(!state.hasObjectFluent(p, &frameid)) {
			ROS_ERROR("%s: object: %s - no frameid in state.", __func__, object.c_str());
			return false;
		}

		pose.header.frame_id = frameid;
		pose.header.stamp = ros::Time(timestamp);
		pose.pose.position.x = posX;
		pose.pose.position.y = posY;
		pose.pose.position.z = posZ;
		pose.pose.orientation.x = qx;
		pose.pose.orientation.y = qy;
		pose.pose.orientation.z = qz;
		pose.pose.orientation.w = qw;

		return true;
	}

	bool comparePoses(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, double tolerance)
	{
		if (fabs(p1.position.x - p2.position.x) >= tolerance)
			return false;
		if (fabs(p1.position.y - p2.position.y) >= tolerance)
			return false;
		if (fabs(p1.position.z - p2.position.z) >= tolerance)
			return false;
		if (fabs(p1.orientation.x - p2.orientation.x) >= tolerance)
			return false;
		if (fabs(p1.orientation.y - p2.orientation.y) >= tolerance)
			return false;
		if (fabs(p1.orientation.z - p2.orientation.z) >= tolerance)
			return false;
		if (fabs(p1.orientation.w - p2.orientation.w) >= tolerance)
			return false;

		return true;
	}

	bool comparePoseStampeds(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2,
			double tolerance)
	{
		if (p1.header.frame_id != p2.header.frame_id)
			return false;
		if (!comparePoses(p1.pose, p2.pose, tolerance))
			return false;

		return true;

	}

}
