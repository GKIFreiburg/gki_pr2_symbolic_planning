#include "object_manipulation_actions/actionExecutorLiftTorsoGrounding.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <symbolic_planning_utils/extractPose.h>
#include <symbolic_planning_utils/joint_limits.h>
//#include <joint_limits_interface/joint_limits_urdf.h>

PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorLiftTorsoGrounding, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
	ActionExecutorLiftTorsoGrounding::ActionExecutorLiftTorsoGrounding()
	{
		torso_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getTorsoGroup();

	    ros::NodeHandle nhPriv("~");
	    // Namespace is "/continual_planning_executive"(/vdist_head_to_table)
	    nhPriv.param("vdist_threshold", vdist_threshold_, 0.002);
	}

	ActionExecutorLiftTorsoGrounding::~ActionExecutorLiftTorsoGrounding()
	{
	}

	void ActionExecutorLiftTorsoGrounding::initialize(const std::deque<std::string> & arguments)
	{
		ROS_ASSERT(arguments.size() == 2);
		action_name_ 			 	= arguments[0];		// lift-torso
		sampled_torso_height_		= arguments[1];
	}

	bool ActionExecutorLiftTorsoGrounding::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
	{
		return a.name == action_name_;
	}

	bool ActionExecutorLiftTorsoGrounding::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
	{
		ROS_ASSERT(a.parameters.size() == 1);
		std::string table        = a.parameters[0];

		// fetch sampled torso height from symbolic state
        Predicate p;
        p.name = sampled_torso_height_;
        double sampled_torso_height;
        if(!currentState.hasNumericalFluent(p, &sampled_torso_height))
        {
        	ROS_ERROR("ActionExecutorLiftTorsoGrounding::%s: Could not fetch '%s' from symbolic state!",
        			__func__, sampled_torso_height_.c_str());
            return false;
        }

        // negative torso height, means it was not initialized, f.ex. no sampling was done.
        if (sampled_torso_height < 0)
        {
        	ROS_WARN("ActionExecutorLiftTorsoGrounding::%s: Torso height was not sampled!", __func__);
        	return true;
        }

	    // get current torso height
		tf::StampedTransform transform;
		try
		{
			tf_.lookupTransform("/map", "/torso_lift_link", ros::Time(0), transform);
		} catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			return false;
		}
		double current_torso_height = transform.getOrigin().z();

		ROS_WARN("ActionExecutorLiftTorsoGrounding::%s: sampled height: %lf, current height: %lf",
				__func__, sampled_torso_height, current_torso_height);

		// Compute difference between both heights and check if it is necessary to lift torso
	    double distance = sampled_torso_height - current_torso_height;
	    ROS_WARN("ActionExecutorLiftTorsoGrounding::%s: distance: %lf, vdist_threshold: %lf",
	    		__func__, fabs(distance), vdist_threshold_);
	    if (fabs(distance) < vdist_threshold_)
	    	return true;

	    // check that distance is smaller than maximum value of torso lift joint value
	    ROS_ASSERT(abs(distance) < 0.33);

	    // fetch current joint value
	    std::vector<double> current_joint_values = torso_group_->getCurrentJointValues();
	    ROS_ASSERT(current_joint_values.size() == 1);
	    const double& current_torso_value = current_joint_values[0];

		ROS_WARN("ActionExecutorLiftTorsoGrounding::%s: current torso joint value: %lf",
				__func__, current_torso_value);

	    // get torso joint name
	    std::vector<std::string> joints = torso_group_->getJoints();
	    ROS_ASSERT(joints.size() == 1);
	    const std::string& joint_name = joints.at(0);

	    double new_joint_value = current_torso_value + distance;

		ROS_WARN("ActionExecutorLiftTorsoGrounding::%s: new torso joint value: %lf",
				__func__, new_joint_value);

	    if (!torso_group_->setJointValueTarget(joint_name, new_joint_value))
		{
			ROS_WARN("ActionExecutorLiftTorsoGrounding::%s: joint %s has value %lf which is out of bound. - RETRYING",
					__func__, joint_name.c_str(), new_joint_value);

			// get torso joint limit
		    symbolic_planning_utils::JointLimits::Limits joint_limits = symbolic_planning_utils::JointLimits::getJointLimit(torso_group_, joint_name);
		    ROS_INFO("ActionExecutorLiftTorsoGrounding::%s: Joint %s has limits [%lf, %lf]",
		    		__func__, joint_name.c_str(), joint_limits.min_position, joint_limits.max_position);

			if (new_joint_value < joint_limits.min_position) // lower than min value
				new_joint_value = joint_limits.min_position;
			else if (new_joint_value > joint_limits.max_position) // higher than max value
				new_joint_value = joint_limits.max_position;

			if (!torso_group_->setJointValueTarget(joint_name, new_joint_value))
			{
				ROS_ERROR("ActionExecutorLiftTorsoGrounding::%s: joint %s is out of bounds", __func__, joint_name.c_str());
				return false;
			}
		}

		// Call the planner to compute a plan.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		moveit::planning_interface::MoveGroup::Plan my_plan;

		moveit::planning_interface::MoveItErrorCode error_code;
        error_code = torso_group_->plan(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ActionExecutorLiftTorsoGrounding::%s: Ups, something with torso motion planning went wrong.", __func__);
			return false;
		}

		// planning was successful
		error_code = torso_group_->execute(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ActionExecutorLiftTorsoGrounding::%s: Ups, something with torso motion execution went wrong.", __func__);
			return false;
		}

		return true;
	}

	void ActionExecutorLiftTorsoGrounding::cancelAction()
	{
	}

}; // namespace

