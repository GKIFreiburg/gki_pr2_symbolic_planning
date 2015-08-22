#include "tidyup_state_creators/goalCreatorTidyupInitializeGrounding.h"
#include "tidyup_utils/geometryPoses.h"
#include "tidyup_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>
#include <moveit_msgs/LoadMap.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorTidyupInitializeGrounding, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{
    GoalCreatorTidyupInitializeGrounding::GoalCreatorTidyupInitializeGrounding()
    {
    }

    GoalCreatorTidyupInitializeGrounding::~GoalCreatorTidyupInitializeGrounding()
    {
    }

    void GoalCreatorTidyupInitializeGrounding::initialize(const std::deque<std::string> & arguments)
    {
    	// fetch octomap path
    	ros::NodeHandle nhPriv("~");
    	std::string octomap_path;
        if(!nhPriv.getParam("octomap_path", octomap_path)) {
            ROS_ERROR("GoalCreatorTidyupInitializeGrounding::%s: Could not get ~octomap_path parameter in namespace: %s.",
            		__func__, nhPriv.getNamespace().c_str());
            return;
        }

    	// send recorded octomap to move_group
        ros::ServiceClient client = nhPriv.serviceClient<moveit_msgs::LoadMap>("/move_group/load_map");
        moveit_msgs::LoadMap srv;
        srv.request.filename = octomap_path;
        if (!client.call(srv))
        {
        	ROS_ERROR("GoalCreatorTidyupInitializeGrounding::%s: Could not send %s to move_group", __func__, octomap_path.c_str());
        }
    }

    bool GoalCreatorTidyupInitializeGrounding::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        ros::NodeHandle nhPriv("~");
        // first add the type hierarchy
        currentState.addSuperType("pose", "pose");
        currentState.addSuperType("frameid", "frameid");
        currentState.addSuperType("location", "pose");
        currentState.addSuperType("table", "pose");
        currentState.addSuperType("movable_object", "pose");
        currentState.addSuperType("arm", "arm");
        currentState.addSuperType("arm_state", "arm_state");

        goal.addSuperType("pose", "pose");
        goal.addSuperType("frameid", "frameid");
        goal.addSuperType("location", "pose");
        goal.addSuperType("table", "pose");
        goal.addSuperType("movable_object", "pose");
        goal.addSuperType("arm", "arm");
        goal.addSuperType("arm_state", "arm_state");

        currentState.printSuperTypes();

        currentState.setObjectFluent("arm-state", "right_arm", "arm_unknown");
        currentState.setObjectFluent("arm-state", "left_arm", "arm_unknown");

        // set sampled-torso-height (only one time needed, else set in grounding modules)
        currentState.setNumericalFluent("sampled-torso-height", "", -1.0);

        return true;
    }

};

