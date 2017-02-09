#include "planner_navigation_actions/actionExecutorROSNavigationGrounding.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <symbolic_planning_utils/joint_limits.h>

#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS(navigation_actions::ActionExecutorROSNavigationGrounding, continual_planning_executive::ActionExecutorInterface)

// Distance measured from ground when torso is at minimum (= not lifted)
#define MIN_TORSO_POSITION 0.802

namespace navigation_actions
{
    void ActionExecutorROSNavigationGrounding::initialize(const std::deque<std::string> & arguments)
    {
        // move-robot-to-table move_base table-inspected-recently sampled-torso-height
        ROS_ASSERT(arguments.size() == 3);
        action_name_ 						= arguments[0];
        action_topic_move_base_				= arguments[1];
        predicate_table_inspected_recently_ = arguments[2];

        action_move_base_  = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(action_topic_move_base_, true);

    	ROS_INFO("ActionExecutorROSNavigationGrounding::%s: Waiting for %s"
    			"action server to start.", __func__, action_topic_move_base_.c_str());
    	action_move_base_->waitForServer();

    	torso_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getTorsoGroup();

		ROS_INFO("ActionExecutorROSNavigationGrounding::%s: Action clients are ready", __func__);
    }

	bool ActionExecutorROSNavigationGrounding::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
		return a.name == action_name_;
	}

	bool ActionExecutorROSNavigationGrounding::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
		ROS_ASSERT(a.parameters.size() == 2);
		std::string table 	 			 = a.parameters[0];
		std::string grounded_target 	 = a.parameters[1];

		// fetch target pose
		geometry_msgs::PoseStamped target_pose;
		target_pose.header.stamp = ros::Time::now();

        std::string name_space = "grounding/drive_pose_module/";
        bool ok = true;
        if (!ros::param::get(name_space + grounded_target + "/x", target_pose.pose.position.x))
        	ok = false;
        if (!ros::param::get(name_space + grounded_target + "/y", target_pose.pose.position.y))
        	ok = false;
        if (!ros::param::get(name_space + grounded_target + "/z", target_pose.pose.position.z))
        	ok = false;
        if (!ros::param::get(name_space + grounded_target + "/qx", target_pose.pose.orientation.x))
        	ok = false;
        if (!ros::param::get(name_space + grounded_target + "/qy", target_pose.pose.orientation.y))
        	ok = false;
        if (!ros::param::get(name_space + grounded_target + "/qz", target_pose.pose.orientation.z))
        	ok = false;
        if (!ros::param::get(name_space + grounded_target + "/qw", target_pose.pose.orientation.w))
        	ok = false;
        if (!ros::param::get(name_space + grounded_target + "/frame_id", target_pose.header.frame_id))
        	ok = false;

        if (!ok)
        {
        	ROS_ERROR("ActionExecutorROSNavigationGrounding::%s: Could not load pose with name: %s",
        			__func__, grounded_target.c_str());
        	return false;
        }
        else
        	ROS_INFO_STREAM("Created goal for ActionExecutorROSNavigationGrounding as: " << grounded_target);

		// set target pose and execute move action
		if (!executeMoveBase(target_pose))
			return false;

		if (!executeLiftTorso(target_pose))
			return false;

		// set inspect-recently to false
		currentState.setBooleanPredicate(predicate_table_inspected_recently_, table, false);

		return true;
	}

	void ActionExecutorROSNavigationGrounding::cancelAction()
	{

	}

	bool ActionExecutorROSNavigationGrounding::executeMoveBase(const geometry_msgs::PoseStamped& target_pose)
	{
		ROS_INFO("ActionExecutorROSNavigationGrounding::%s: Sending move base request.", __func__);
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose = target_pose;

		action_move_base_->sendGoal(goal);

		action_move_base_->waitForResult();

		actionlib::SimpleClientGoalState state = action_move_base_->getState();
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			// no need to check for result, since there is no result
			ROS_INFO("ActionExecutorROSNavigationGrounding::%s: Move Base Action finished.", __func__);
			return true;
		}

		ROS_ERROR("ActionExecutorROSNavigationGrounding::%s: Move Base Action failed.", __func__);
		return false;
	}

	bool ActionExecutorROSNavigationGrounding::executeLiftTorso(const geometry_msgs::PoseStamped& target_pose)
	{
		double joint_value = target_pose.pose.position.z - MIN_TORSO_POSITION;
		ROS_INFO("ActionExecutorROSNavigationGrounding::%s: target height: %lf -> joint_value: %lf",
				__func__, target_pose.pose.position.z, joint_value);

	    // get torso joint name
	    std::vector<std::string> joints = torso_group_->getJoints();
	    ROS_ASSERT(joints.size() == 1);
	    std::string joint_name = joints.at(0);

	    if (!torso_group_->setJointValueTarget(joint_name, joint_value))
		{
			ROS_WARN("ActionExecutorROSNavigationGrounding::%s: joint %s has value %lf which is out of bound. - RETRYING",
					__func__, joint_name.c_str(), joint_value);

			// get torso joint limit
		    symbolic_planning_utils::JointLimits::Limits joint_limits = symbolic_planning_utils::JointLimits::getJointLimit(torso_group_, joint_name);
		    ROS_INFO("ActionExecutorROSNavigationGrounding::%s: Joint %s has limits [%lf, %lf]",
		    		__func__, joint_name.c_str(), joint_limits.min_position, joint_limits.max_position);

			if (joint_value < joint_limits.min_position) // lower than min value
				joint_value = joint_limits.min_position;
			else if (joint_value > joint_limits.max_position) // higher than max value
				joint_value = joint_limits.max_position;

			if (!torso_group_->setJointValueTarget(joint_name, joint_value))
			{
				ROS_ERROR("ActionExecutorROSNavigationGrounding::%s: joint %s is out of bounds", __func__, joint_name.c_str());
				return false;
			}
		}

		moveit::planning_interface::MoveItErrorCode error_code;
		// Call the planner to compute a plan.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		moveit::planning_interface::MoveGroup::Plan my_plan;

		for (int exec_count = 0; exec_count < 5; exec_count)
		{
			ROS_DEBUG("ActionExecutorROSNavigationGrounding::%s: planning torso motion...", __func__);
			error_code = torso_group_->plan(my_plan);
			if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
			{
				ROS_ERROR_STREAM("ActionExecutorROSNavigationGrounding::"<<__func__<<" planning failed: "<<error_code);
				continue;
			}

			ROS_DEBUG("ActionExecutorROSNavigationGrounding::%s: executing torso motion...", __func__);
			error_code = torso_group_->execute(my_plan);
			if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
			{
				// planning was successful
				ROS_INFO("ActionExecutorROSNavigationGrounding::%s: Lift Torso Action finished.", __func__);
				return true;
			}
		}

		ROS_ERROR_STREAM("ActionExecutorROSNavigationGrounding::"<<__func__<<" execution failed: "<<error_code);
		return false;
	}
};

