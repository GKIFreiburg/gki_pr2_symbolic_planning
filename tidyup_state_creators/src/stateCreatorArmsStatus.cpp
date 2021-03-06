#include "tidyup_state_creators/stateCreatorArmsStatus.h"
#include <pluginlib/class_list_macros.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <tidyup_utils/stringutil.h>
#include <tidyup_utils/get_pose_stamped_from_param.h>
#include <cmath>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorArmsStatus, continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{
    StateCreatorArmsStatus::StateCreatorArmsStatus()
    {
    }

    StateCreatorArmsStatus::~StateCreatorArmsStatus()
    {
    }

    void StateCreatorArmsStatus::initialize(const std::deque<std::string> & arguments)
    {
        ROS_ASSERT(arguments.size() == 8);
        named_target_right_arm_to_side_  = arguments[0];		// right_arm_to_side
        named_target_left_arm_to_side_   = arguments[1];		// left_arm_to_side
        named_target_right_arm_to_front_ = arguments[2];		// right_arm_to_front
        named_target_left_arm_to_front_  = arguments[3];		// left_arm_to_front

        predicate_name_     	   		 = arguments[4];		// arm-state
        predicate_at_side_  	   		 = arguments[5];		// arm_at_side
        predicate_at_front_ 	   		 = arguments[6];		// arm_at_front
        predicate_unkown_   	   		 = arguments[7];		// arm_unkown

        right_arm_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
        left_arm_  = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();

        tolerance_ = 0.02;
    }

    bool StateCreatorArmsStatus::fillState(SymbolicState & state)
    {
    	setArmStatusInSymbolicState(state, right_arm_);
    	setArmStatusInSymbolicState(state, left_arm_);
        return true;
    }

    void StateCreatorArmsStatus::setArmStatusInSymbolicState(
    		SymbolicState& state, moveit::planning_interface::MoveGroup* group)
    {
    	std::string target_side;
    	std::string target_front;
    	if (StringUtil::startsWith(group->getName(), "right_"))
    	{
    		target_side  = named_target_right_arm_to_side_;
    		target_front = named_target_right_arm_to_front_;
    	}
    	else if (StringUtil::startsWith(group->getName(), "left_"))
    	{
    		target_side  = named_target_left_arm_to_side_;
    		target_front = named_target_left_arm_to_front_;
    	}
    	else
    	{
    		ROS_ERROR("StateCreatorArmsStatus::%s: Could not determine named target", __func__);
    		return;
    	}

    	if (checkIfArmInTargetPosition(group, target_side))
    	{
    		state.setObjectFluent(predicate_name_, group->getName(), predicate_at_side_);
    		return;
    	}

    	if (checkIfArmInTargetPosition(group, target_front))
    	{
    		state.setObjectFluent(predicate_name_, group->getName(), predicate_at_front_);
    		return;
    	}

    	state.setObjectFluent(predicate_name_, group->getName(), predicate_unkown_);
    }

    bool StateCreatorArmsStatus::checkIfArmInTargetPosition(
    		moveit::planning_interface::MoveGroup* group, const std::string& target)
    {
    	// set current_joint_values
		std::vector<double> current_joint_values = group->getCurrentJointValues();
		std::vector<std::string> joint_names = group->getJoints();

		// set target_joint_values
		std::vector<double> target_joint_values;
		ROS_ASSERT(target_joint_values.size() == 0);
		geometry_msgs::PoseStamped pose;

		// arms_at_side is a named target in the srdf file (since it is defined using joint values)
		if (group->setNamedTarget(target)) // arms_at_side
		{
			const robot_state::RobotState& rs = group->getJointValueTarget();
			const robot_state::JointModelGroup* joint_model_group = rs.getJointModelGroup(group->getName());
			rs.copyJointGroupPositions(joint_model_group, target_joint_values);

	  		ROS_ASSERT(current_joint_values.size() == target_joint_values.size());

	  		// check the current joint positions against the predefined positions in the srdf file
	  		for (int i = 0; i < current_joint_values.size(); i++)
	  		{
	  			// normalize values
	  			normalizeJointValue(current_joint_values[i]);
	  			normalizeJointValue(target_joint_values[i]);

//	  			ROS_INFO("StateCreatorArmsStatus::%s: Target: %s || %s [current] - [target] : %lf - %lf",__func__, target.c_str(),
//	  					 joint_names[i].c_str(),
//	  					 current_joint_values[i], target_joint_values[i]);
				// tolerance needed since joints never reach exactly the given values.
	  			if (std::fabs((double)current_joint_values[i] - (double)target_joint_values[i]) > tolerance_)
	  				return false;
	  		}
	    	return true;
		}

		// only valid for arms_at_front
		// fetch desired at_to_front pose from param server
		else if (tidyup_utils::getPoseStampedFromParam(target, pose))
		{
			geometry_msgs::PoseStamped current_pose = group->getCurrentPose();

			// transform pose from param to the same frame as the current pose
			geometry_msgs::PoseStamped pose_transformed;
			try {
				tf_.waitForTransform(current_pose.header.frame_id, pose.header.frame_id, pose.header.stamp,
						ros::Duration(0.5));
				tf_.transformPose(current_pose.header.frame_id, pose, pose_transformed);
			} catch (tf::TransformException& ex) {
				ROS_ERROR("%s", ex.what());
				return false;
			}

			// compute the difference between both poses, by computing the inverse transform
			tf::Transform current, target, result;
			tf::poseMsgToTF(pose_transformed.pose, target);
			tf::poseMsgToTF(current_pose.pose, current);

			result = current.inverseTimes(target);
			geometry_msgs::Pose res;
			tf::poseTFToMsg(result, res);

			bool ret = true;

			// check if difference is smaller than tolerance value
			if (res.position.x > tolerance_)
				ret = false;
			if (res.position.y > tolerance_)
				ret = false;
			if (res.position.z > tolerance_)
				ret = false;
			if (res.orientation.x > tolerance_)
				ret = false;
			if (res.orientation.y > tolerance_)
				ret = false;
			if (res.orientation.z > tolerance_)
				ret = false;
			// w is always 1, do not need to verify
//			if (res.orientation.w > tolerance_)
//				ret = false;

			return ret;
		}

		ROS_ERROR("StateCreatorArmsStatus::%s: Could not determine target joint values", __func__);
		return false;
    }

    void StateCreatorArmsStatus::normalizeJointValue(double& jointValue)
    {
		// taking care of joints that can rotate 360 degrees (continuous type)
		// affected joints are: *_fore_arm_roll_joint and *_wrist_roll_joint
		// joint values should be in the range of [-PI, +PI]
		while (fabs(jointValue) > M_PI)
		{
//			ROS_INFO("StateCreatorArmsStatus::%s: joint Value: %lf", __func__, jointValue);
			if (jointValue > 0) // positive
				jointValue = jointValue - 2*M_PI;
			else // negative
				jointValue = jointValue + 2*M_PI;
//			ROS_INFO("StateCreatorArmsStatus::%s: normalized joint Value: %lf", __func__, jointValue);
		}
    }
};

