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

        if(arguments.size() < 3)
            return;

        bool parseStart = true;
        if(arguments[2] == "goal") {
            parseStart = false;
        } else {
            ROS_ASSERT(arguments[2] == "start");
        }

        unsigned int curArg = 3;
        while(curArg < arguments.size()) {
            if(arguments[curArg] == "goal") {
                parseStart = false;
                curArg++;
                continue;
            }
            ROS_ASSERT(arguments.size() >= curArg + 2);  // need to access curArg, curArg+1
            string pred = arguments[curArg];
            string setS = arguments[curArg + 1];
            bool set = false;
            if(setS == "true")
                set = true;
            else if(setS == "false")
                set = false;
            else
                ROS_ASSERT(false);
            if(parseStart)
                _startPredicates.push_back(std::make_pair(pred, set));
            else
                _goalPredicates.push_back(std::make_pair(pred, set));
            curArg += 2;
        }
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

        std::string name_space = "tfd_modules/drive_pose/";
        if (!ros::param::get(name_space + grounded_surface_name + "/x", goal.target_pose.pose.position.x))
        	return false;
        if (!ros::param::get(name_space + grounded_surface_name + "/y", goal.target_pose.pose.position.y))
			return false;
        if (!ros::param::get(name_space + grounded_surface_name + "/z", goal.target_pose.pose.position.z))
        			return false;
        if (!ros::param::get(name_space + grounded_surface_name + "/qx", goal.target_pose.pose.orientation.x))
        	return false;
        if (!ros::param::get(name_space + grounded_surface_name + "/qy", goal.target_pose.pose.orientation.y))
			return false;
        if (!ros::param::get(name_space + grounded_surface_name + "/qz", goal.target_pose.pose.orientation.z))
			return false;
        if (!ros::param::get(name_space + grounded_surface_name + "/qw", goal.target_pose.pose.orientation.w))
			return false;
        if (!ros::param::get(name_space + grounded_surface_name + "/frame_id", goal.target_pose.header.frame_id))
			return false;

        ROS_INFO_STREAM("Created goal for ActionExecutorROSNavigationGrounding as: " << goal);

        return true;
    }

    void ActionExecutorROSNavigationGrounding::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const move_base_msgs::MoveBaseResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 2);
        string surface_name 		  = a.parameters[0];
        string grounded_surface_name  = a.parameters[1];

        // start predicates are always applied independent of success
        for(std::vector<std::pair<std::string, bool> >::iterator it = _startPredicates.begin();
                it != _startPredicates.end(); it++) {
            current.setBooleanPredicate(it->first, surface_name, it->second);
        }

//        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
//            for(std::vector<std::pair<std::string, bool> >::iterator it = _goalPredicates.begin();
//                    it != _goalPredicates.end(); it++) {
//                current.setBooleanPredicate(it->first, targetName, it->second);
//            }
//        }
    }

};

