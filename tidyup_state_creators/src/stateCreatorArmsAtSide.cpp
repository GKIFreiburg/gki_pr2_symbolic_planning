#include "tidyup_state_creators/stateCreatorArmsAtSide.h"
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_DECLARE_CLASS(tidyup_state_creators, state_creator_arms_at_side,
//        tidyup_state_creators::StateCreatorArmsAtSide, continual_planning_executive::StateCreator)
PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorArmsAtSide, continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{
    StateCreatorArmsAtSide::StateCreatorArmsAtSide()
    {
    }
    StateCreatorArmsAtSide::~StateCreatorArmsAtSide()
    {
    }
    void StateCreatorArmsAtSide::initialize(const std::deque<std::string> & arguments)
    {
        ROS_ASSERT(arguments.size() == 3);
        service_name = arguments[0];		// /tidyup/arms_at_side
        predicate_name = arguments[1];		// arm-state
        predicate_value = arguments[2]; 	// arm_at_side
        ros::NodeHandle nh;
        client = nh.serviceClient<tidyup_msgs::ArmsAtSide>(service_name);
        client.waitForExistence();
    }

    bool StateCreatorArmsAtSide::fillState(SymbolicState & state)
    {
        tidyup_msgs::ArmsAtSide srv;
        if (! client.call(srv))
        {
            ROS_ERROR("service call to %s failed!", service_name.c_str());
            return false;
        }
        if (srv.response.right_arm)
            state.setObjectFluent(predicate_name, "right_arm", predicate_value);
        if (srv.response.left_arm)
            state.setObjectFluent(predicate_name, "left_arm", predicate_value);
        return true;
    }
};

