#include "planner_modules_pr2/lift_torso_module.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

//#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

VERIFY_CONDITIONCHECKER_DEF(lift_torso_cost);

void lift_torso_init(int argc, char** argv)
{
    ROS_INFO("%s: Initialized lift_torso Module.", __func__);
}

double lift_torso_cost(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	double cost;
	cost = 12.5;
    return cost;
}

