#include "object_manipulation_actions/LiftTorso.h"

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <symbolic_planning_utils/extractPose.h>
#include <symbolic_planning_utils/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::LiftTorso,
		continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
LiftTorso::LiftTorso()
{
	torso_group_ =
			symbolic_planning_utils::MoveGroupInterface::getInstance()->getTorsoGroup();

	ros::NodeHandle nhPriv("~");
	// Namespace is "/continual_planning_executive"(/vdist_head_to_table)
	nhPriv.param("vdist_head_to_table", vdist_head_to_table_, 0.60);
	nhPriv.param("vdist_threshold", vdist_threshold_, 0.002);
	nhPriv.param("min_torso_vel", min_torso_vel_, 0.0001);
	nhPriv.param("stallThreshold", stallThreshold_, 2);
}

LiftTorso::~LiftTorso()
{

}

void LiftTorso::initialize(const std::deque<std::string> & arguments)
{
	ROS_ASSERT(arguments.size() == 1);
	action_name_ = arguments[0];		// lift-torso
}

bool LiftTorso::canExecute(const DurativeAction & a,
		const SymbolicState & currentState) const
{
	return a.name == action_name_;
}

bool LiftTorso::executeBlocking(const DurativeAction & a,
		SymbolicState & currentState)
{
	ROS_ASSERT(a.parameters.size() == 2);
	std::string table = a.parameters[0];
	std::string mani_loc = a.parameters[1];

	// get tablePose
	geometry_msgs::PoseStamped tablePose;
	if (!symbolic_planning_utils::extractPoseStampedFromSymbolicState(
			currentState, table, tablePose))
	{
		ROS_ERROR_STREAM(
				action_name_<<": Could not extract pose of table: "<<table<<" from symbolic state");
		return false;
	}

	moveit::planning_interface::MoveItErrorCode error_code;
	error_code = executeLiftTorso(tablePose);

	return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

void LiftTorso::cancelAction()
{

}

moveit::planning_interface::MoveItErrorCode LiftTorso::executeLiftTorso(
		const geometry_msgs::PoseStamped tablePose)
{
	// compute difference in height between head_mount_link and table
	// transform tablePose into frame of /base link
	//ROS_ASSERT(tablePose.header.frame_id == "/map");

	ROS_DEBUG_STREAM(action_name_<<": table pose" << tablePose);

	// transform headPose into frame of /map
	geometry_msgs::PoseStamped headPose;
	headPose.header.frame_id = "/head_pan_link";
	headPose.header.stamp = ros::Time::now();
	geometry_msgs::Quaternion q;
	q.x = 0;
	q.y = 0;
	q.z = 0;
	q.w = 1;
	headPose.pose.orientation = q;
	geometry_msgs::Point p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	headPose.pose.position = p;

	geometry_msgs::PoseStamped head_transformed;
	try
	{
		tf_.waitForTransform("/map", headPose.header.frame_id,
				headPose.header.stamp, ros::Duration(0.5));
		tf_.transformPose("/map", headPose, head_transformed);
	} catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
		return false;
	}
	ROS_DEBUG_STREAM(action_name_<<": head pose transformed" << head_transformed);

	double distance = fabs(
			head_transformed.pose.position.z - tablePose.pose.position.z);
	ROS_DEBUG("Distance: %lf, head: %lf, table: %lf", distance,
			head_transformed.pose.position.z, tablePose.pose.position.z);
	if (vdist_head_to_table_ - vdist_threshold_ < distance
			&& distance < vdist_head_to_table_ + vdist_threshold_)
		// head_mount_link is about vdist_head_to_table_ above table, no need to lift torso
		return moveit::planning_interface::MoveItErrorCode::SUCCESS;

	// get torso position
	std::vector<double> current_joint_values =
			torso_group_->getCurrentJointValues();
	ROS_ASSERT(current_joint_values.size() == 1);
	double torso_position = current_joint_values[0];

	// get torso joint name
	std::vector<std::string> joints = torso_group_->getJoints();
	ROS_ASSERT(joints.size() == 1);
	std::string joint_name = joints.at(0);

	torso_position = torso_position + vdist_head_to_table_ - distance;
	if (!torso_group_->setJointValueTarget(joint_name, torso_position))
	{
		ROS_WARN_STREAM(action_name_<<": joint "<<joint_name<<" has value "<<torso_position<<" which is out of bound. - RETRYING");

		// get torso joint limit
		symbolic_planning_utils::JointLimits::Limits joint_limits =
				symbolic_planning_utils::JointLimits::getJointLimit(torso_group_,
						joint_name);
		ROS_INFO_STREAM(action_name_<<": joint "<<joint_name<<" has limits ["<<joint_limits.min_position<<", "<<joint_limits.max_position<<"]");

		if (torso_position < joint_limits.min_position) // lower than min value
			torso_position = joint_limits.min_position;
		else if (torso_position > joint_limits.max_position) // higher than max value
			torso_position = joint_limits.max_position;

		if (!torso_group_->setJointValueTarget(joint_name, torso_position))
		{
			ROS_ERROR("ActionExecutorLiftTorso::%s: joint %s is out of bounds",
					__func__, joint_name.c_str());
			return moveit::planning_interface::MoveItErrorCode::FAILURE;
		}
	}

	return symbolic_planning_utils::MoveGroupInterface::getInstance()->planExecuteVerify(torso_group_, action_name_);
}

}
;

