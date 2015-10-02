#include "object_manipulation_actions/actionExecutorArmToFront.h"
#include <pluginlib/class_list_macros.h>
#include <tidyup_utils/stringutil.h>
#include <tidyup_utils/get_pose_stamped_from_param.h>
#include <symbolic_planning_utils/moveGroupInterface.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>
#include <istream>
#include <ostream>
#include <fstream>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorArmToFront, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{

	void ActionExecutorArmToFront::initialize(const std::deque<std::string> & arguments)
	{
		ROS_ASSERT(arguments.size() == 3);

		actionName_ 				 = arguments[0]; 		// arm-to-front
		rosparam_right_arm_to_front_ = arguments[1];		// right_arm_to_front
		rosparam_left_arm_to_front_  = arguments[2];		// left_arm_to_front

		ros::NodeHandle n;
		pub_pose_ = n.advertise<geometry_msgs::PoseStamped>("arm_to_front", 10, true);

		left_arm_ = new moveit::planning_interface::MoveGroup("left_arm");
		right_arm_ = new moveit::planning_interface::MoveGroup("right_arm");
		head_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getHeadGroup();

		joint_name_head_yaw_ = "head_pan_joint";
	}

	bool ActionExecutorArmToFront::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
	    return a.name == actionName_;
	}

	bool ActionExecutorArmToFront::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		// Point head straight forward
		// first remember actual joint position
		// store the joint value for the pointed head position
		std::vector<double> current_joint_values = head_group_->getCurrentJointValues();
		std::vector<std::string> joint_names = head_group_->getJoints();
		double pointed_head_joint_value;

		ROS_ASSERT(current_joint_values.size() == joint_names.size());
		for (size_t i = 0; i < current_joint_values.size(); i++)
		{
			if (joint_names[i] == joint_name_head_yaw_)
				pointed_head_joint_value = current_joint_values[i];
		}

		// move head to 0 position
		if (!head_group_->setJointValueTarget(joint_name_head_yaw_, 0))
			ROS_ERROR("ActionExecutorArmToFront::%s: joint values out of bounds!", __func__);
		error_code = head_group_->move();
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
			ROS_ERROR("ActionExecutorArmToFront::%s: Could not move head to initial position!", __func__);

		// Move arm in front of camera
		ROS_ASSERT(a.parameters.size() == 1);
		moveit::planning_interface::MoveGroup* arm_group;
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "head_mount_kinect_rgb_link";
		pose.header.stamp = ros::Time::now();

		if (StringUtil::startsWith(a.parameters[0], "left_"))
		{
//			arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
			arm_group = left_arm_;
			if (!tidyup_utils::getPoseStampedFromParam(rosparam_left_arm_to_front_, pose))
			{
				ROS_WARN("ActionExecutorArmToFront::%s: Could not load arm configuration from param!", __func__);
				pose.pose.position.x = 0.48;
				// distance from tool_frame to end_effector link - could be read from tf, but it is a fixed distance
				pose.pose.position.y = 0.2;
				pose.pose.position.z = 0.0;

				tf::Quaternion tf_q;
				tf_q.setRPY(0, 0, -M_PI/2);
				tf::quaternionTFToMsg(tf_q, pose.pose.orientation);
			}
		}
		else if (StringUtil::startsWith(a.parameters[0], "right_"))
		{
//			arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
			arm_group = right_arm_;
			if (!tidyup_utils::getPoseStampedFromParam(rosparam_right_arm_to_front_, pose))
			{
				ROS_WARN("ActionExecutorArmToFront::%s: Could not load arm configuration from param!", __func__);
				pose.pose.position.x = 0.48;
				pose.pose.position.y = -0.2;
				pose.pose.position.z = 0;

				tf::Quaternion tf_q;
				geometry_msgs::Quaternion geo_q;
				tf_q.setRPY(0, 0, M_PI/2);
				tf::quaternionTFToMsg(tf_q, pose.pose.orientation);
			}
		}
		else
		{
			ROS_ERROR("ActionExecutorArmToFront::%s: Wrong input from action!", __func__);
			return false;
		}

		// for debug purposes
		ROS_INFO("ActionExecutorArmToFront::%s: Pose published on topic: %s", __func__, pub_pose_.getTopic().c_str());
		pub_pose_.publish(pose);
		// ROS_INFO_STREAM("ActionExecutorArmToFront::" << __func__ << ": Pose: " << pose);

		error_code = executeArmToFront(arm_group, pose);
		if (!error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_ERROR("ActionExecutorArmToFront::%s: Could not move arm in front of camera!", __func__);
			return false;
		}

//		// move head back
//		if (!head_group_->setJointValueTarget(joint_name_head_yaw_, pointed_head_joint_value))
//			ROS_ERROR("ActionExecutorArmToFront::%s: joint values out of bounds!", __func__);
//		error_code = head_group_->move();
//		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
//		{
//			ROS_ERROR("ActionExecutorArmToFront::%s: Could not move head to initial position!", __func__);
//			return false;
//		}

		return true;
	}

	void ActionExecutorArmToFront::cancelAction()
	{

	}

	moveit::planning_interface::MoveItErrorCode ActionExecutorArmToFront::executeArmToFront(
			moveit::planning_interface::MoveGroup* group,
			const geometry_msgs::PoseStamped& pose)
	{
		// Set hard coded pose as target
		if (!group->setPoseTarget(pose))
		{
			ROS_ERROR("ActionExecutorArmToFront::%s: Could not set pose target. \n", __func__);
			return  moveit::planning_interface::MoveItErrorCode::FAILURE;
		}

		moveit::planning_interface::MoveItErrorCode error_code;
		// Call the planner to compute a plan.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		moveit::planning_interface::MoveGroup::Plan my_plan;
		ROS_DEBUG("ActionExecutorArmToFront::%s: planning arm motion...", __func__);

        error_code = group->plan(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ActionExecutorArmToFront::%s: Ups, something with arm motion planning went wrong.\n"
					"Verify that head is inclined - pointing downwards, otherwise pose is not reachable with arm.", __func__);
			return error_code;
		}

		// planning was successful
		ROS_DEBUG("ActionExecutorArmToFront::%s: executing arm motion...", __func__);
		error_code = group->execute(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ActionExecutorArmToFront::%s: Ups, something with arm motion execution went wrong.\n"
					"Verify that head is inclined - pointing downwards, otherwise pose is not reachable with arm.", __func__);
			return error_code;
		}

		return error_code;
	}
};

