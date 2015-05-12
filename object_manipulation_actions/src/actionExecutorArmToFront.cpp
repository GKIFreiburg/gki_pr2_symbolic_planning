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
	}

	bool ActionExecutorArmToFront::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
	    return a.name == actionName_;
	}

	bool ActionExecutorArmToFront::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
		// DEFAULT_RIGHT_ARM_INSPECT_POSE = PoseStamped(Header(frame_id='/head_mount_kinect_rgb_link'),Pose(Point(0.48, -0.2, 0.0), Quaternion(-0.037, -0.031, 0.609, 0.792)))
		// DEFAULT_LEFT_ARM_INSPECT_POSE = PoseStamped(Header(frame_id='/head_mount_kinect_rgb_link'),Pose(Point(0.48, 0.2, 0.0), Quaternion(-0.073, -0.047, -0.669, 0.738)))

		ROS_ASSERT(a.parameters.size() == 1);
		moveit::planning_interface::MoveGroup* arm_group;
		moveit::planning_interface::MoveItErrorCode error_code;
		geometry_msgs::PoseStamped pose;
		std::string target;

		if (StringUtil::startsWith(a.parameters[0], "left_"))
		{
			arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
			target = rosparam_left_arm_to_front_;
			if (!tidyup_utils::getPoseStampedFromParam(rosparam_left_arm_to_front_, pose))
			{
				ROS_WARN("ActionExecutorArmToFront::%s: Could not load pose from param, take hardcoded values!", __func__);
				pose.pose.position.x = 0.48;
				pose.pose.position.y = 0.2;
				pose.pose.position.z = 0;

				tf::Quaternion tf_q;
				geometry_msgs::Quaternion geo_q;
				tf_q.setRPY(45, 0, -90);
				tf::quaternionTFToMsg(tf_q, geo_q);

				ROS_WARN_STREAM("ActionExecutorArmToFront::%s: End effector pose:" << pose);
			}
		}
		else if (StringUtil::startsWith(a.parameters[0], "right_"))
		{
			arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
			target = rosparam_right_arm_to_front_;
			if (!tidyup_utils::getPoseStampedFromParam(rosparam_right_arm_to_front_, pose))
			{
				ROS_WARN("ActionExecutorArmToFront::%s: Could not load pose from param, take hardcoded values!", __func__);
				pose.pose.position.x = 0.48;
				pose.pose.position.y = -0.2;
				pose.pose.position.z = 0;

				tf::Quaternion tf_q;
				geometry_msgs::Quaternion geo_q;
				tf_q.setRPY(-45, 0, 90);
				tf::quaternionTFToMsg(tf_q, geo_q);
				pose.pose.orientation = geo_q;

				ROS_WARN_STREAM("ActionExecutorArmToFront::%s: End effector pose:" << pose);
			}
		}

		// for debug purposes
		pub_pose_.publish(pose);

		error_code = executeArmToFront(arm_group, pose, target);
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	void ActionExecutorArmToFront::cancelAction()
	{

	}

	moveit::planning_interface::MoveItErrorCode ActionExecutorArmToFront::executeArmToFront(
			moveit::planning_interface::MoveGroup* group,
			const geometry_msgs::PoseStamped& pose,
			const std::string target)
	{
		// Set hard coded pose as target
		if (!group->setPoseTarget(pose))
		{
			ROS_ERROR("ActionExecutorArmToFront::%s: Could not set pose target.", __func__);
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
			ROS_WARN("ActionExecutorArmToFront::%s: Ups, something with arm motion planning went wrong.", __func__);
			return error_code;
		}

		// planning was successful
		ROS_DEBUG("ActionExecutorArmToFront::%s: executing arm motion...", __func__);
		error_code = group->execute(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ActionExecutorArmToFront::%s: Ups, something with arm motion execution went wrong.", __func__);
			return error_code;
		}

//		group->rememberJointValues(target);

		return error_code;
	}
};

