#include "planner_navigation_actions/actionExecutorROSNavigationGrounding.h"
#include <pluginlib/class_list_macros.h>

#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS(navigation_actions::ActionExecutorROSNavigationGrounding, continual_planning_executive::ActionExecutorInterface)

namespace navigation_actions
{

    void ActionExecutorROSNavigationGrounding::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorActionlib<move_base_msgs::MoveBaseAction, move_base_msgs::MoveBaseGoal,
            move_base_msgs::MoveBaseResult>::initialize(arguments);

        // move-robot-to-table move_base table-inspected-recently sampled-torso-height
        ROS_ASSERT(arguments.size() == 4);
        predicate_table_inspected_recently_ = arguments[2];
        sampled_torso_height_ 				= arguments[3];
    }

    bool ActionExecutorROSNavigationGrounding::fillGoal(move_base_msgs::MoveBaseGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        // The frame_id should be a fixed frame anyways
        goal.target_pose.header.stamp = ros::Time::now();

        // grounded action
        ROS_ASSERT(a.parameters.size() == 2);
        string surface_name 		  = a.parameters[0];
        string grounded_surface_name  = a.parameters[1];

        ROS_INFO("ActionExecutorROSNavigationGrounding::%s: surface: %s, grounded surface name: %s",
        		__func__, surface_name.c_str(), grounded_surface_name.c_str());

        std::string name_space = "grounding/drive_pose_module/";
        bool ret = true;
        if (!ros::param::get(name_space + grounded_surface_name + "/x", goal.target_pose.pose.position.x))
        	ret = false;
        if (!ros::param::get(name_space + grounded_surface_name + "/y", goal.target_pose.pose.position.y))
        	ret = false;
        if (!ros::param::get(name_space + grounded_surface_name + "/z", goal.target_pose.pose.position.z))
        	ret = false;
        if (!ros::param::get(name_space + grounded_surface_name + "/qx", goal.target_pose.pose.orientation.x))
        	ret = false;
        if (!ros::param::get(name_space + grounded_surface_name + "/qy", goal.target_pose.pose.orientation.y))
        	ret = false;
        if (!ros::param::get(name_space + grounded_surface_name + "/qz", goal.target_pose.pose.orientation.z))
        	ret = false;
        if (!ros::param::get(name_space + grounded_surface_name + "/qw", goal.target_pose.pose.orientation.w))
        	ret = false;
        if (!ros::param::get(name_space + grounded_surface_name + "/frame_id", goal.target_pose.header.frame_id))
        	ret = false;

        if (!ret)
        	ROS_ERROR("ActionExecutorROSNavigationGrounding::%s: Could not load pose with name: %s",
        			__func__, grounded_surface_name.c_str());
        else
        	ROS_INFO_STREAM("Created goal for ActionExecutorROSNavigationGrounding as: " << goal);

        return ret;
    }

    void ActionExecutorROSNavigationGrounding::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const move_base_msgs::MoveBaseResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 2);
        string surface_name 		  = a.parameters[0];
        string grounded_surface_name  = a.parameters[1];

        // as soon as a drive action is executed, table-inspected-recently is set to false for all
        // table elements
    	string table_name;
    	pair<SymbolicState::TypedObjectConstIterator,
    			SymbolicState::TypedObjectConstIterator> targets =
    			current.getTypedObjects().equal_range("table");
    	for (SymbolicState::TypedObjectConstIterator it = targets.first;
    			it != targets.second; it++)
    	{
    		table_name = it->second;
    		current.setBooleanPredicate(predicate_table_inspected_recently_, table_name, false);
    	}

    	// set sampled torso height in symbolic state
    	std::string name_space = "grounding/drive_pose_module/";
    	double torso_height;
        if (!ros::param::get(name_space + grounded_surface_name + "/z", torso_height))
        {
        	ROS_ERROR("ActionExecutorROSNavigationGrounding::%s: Could not set sampled torso height!", __func__);
        	return;
        }
    	current.setNumericalFluent(sampled_torso_height_, "", torso_height);

    }

};

