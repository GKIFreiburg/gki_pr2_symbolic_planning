#include "planner_modules_pr2/lift_torso_module.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

//#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

VERIFY_CONDITIONCHECKER_DEF(liftTorsoCost);
VERIFY_CONDITIONCHECKER_DEF(needToLiftTorso);

// ________________________________________________________________________________________________
bool fetchVariablesFromPlanner(const modules::ParameterList & parameterList,
		modules::numericalFluentCallbackType numericalFluentCallback)
{
	// get table name from planner interface
	ROS_ASSERT(parameterList.size() == 1);

	modules::ParameterList table;
	table.push_back(parameterList[0]);

	modules::ParameterList empty;

	// fetch values from planner
	modules::NumericalFluentList nfRequest;
	nfRequest.reserve(2);
	nfRequest.push_back(modules::NumericalFluent("z", table));
	nfRequest.push_back(modules::NumericalFluent("torso-position", empty));

    modules::NumericalFluentList* nfRequestP = &nfRequest;
    if(!numericalFluentCallback(nfRequestP)) {
        ROS_ERROR("lift_torso_modules::%s: numericalFluentCallback failed.", __func__);
        return false;
    }

    // initialize values
    table_height_ 	= nfRequest[0].value;
    torso_position_ = nfRequest[1].value;

    ROS_INFO("lift_torso_modules::%s: table height: %lf and torso_position: %lf",
    		__func__, table_height_, torso_position_);

    return true;
}

// ________________________________________________________________________________________________
double computeHeadTableDistance()
{
	double head_height = MIN_TORSO_POSITION + torso_position_ + OFFSET_TORSO_HEAD;
	return head_height - table_height_;
}

// ________________________________________________________________________________________________
double computeLiftDistance()
{
	double distance = computeHeadTableDistance();

	// Compute height that torso needs to be lifted
	double lift_distance = torso_position_ + vdist_head_to_table_ - distance;

	// Verify that lift_distance does not leave joint bounds, else set to the corresponding limit
	if (lift_distance < MIN_TORSO_JOINT)
		lift_distance = MIN_TORSO_JOINT;
	else if (lift_distance > MAX_TORSO_JOINT)
		lift_distance = MAX_TORSO_JOINT;

	return lift_distance;
}

// ________________________________________________________________________________________________
void liftTorsoInit(int argc, char** argv)
{
	ROS_ASSERT(argc == 2);

    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);
    world_frame_ = tf::resolve(tfPrefix, argv[1]);

    // /continual_planning_executive/vdist_head_to_table)
    nhPriv.param("/continual_planning_executive/vdist_head_to_table", vdist_head_to_table_, 0.62);
    nhPriv.param("/continual_planning_executive/vdist_threshold", vdist_threshold_, 0.002);

    // If not defined on param server, take 0.02 m/s which is an arbitrary chosen value
    nhPriv.param("lift_speed", lift_speed_, 0.02);

   	ROS_INFO_STREAM("lift_torso_modules::" << __func__ << ": param namespace: " << nhPriv.getNamespace() << "\n"
		"world frame: " << world_frame_ << "\n"
		"vdist_head_to_table: " << vdist_head_to_table_ << "\n"
		"lift_speed: " << lift_speed_);


    ROS_INFO("lift_torso_modules::%s: Initialized lift_torso Module.", __func__);
}

// ________________________________________________________________________________________________
double liftTorsoCost(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{

	if (!fetchVariablesFromPlanner(parameterList, numericalFluentCallback))
		return INFINITE_COST;

	double cost = computeLiftDistance();

	return cost/lift_speed_;
}

// ________________________________________________________________________________________________
double needToLiftTorso(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	if (!fetchVariablesFromPlanner(parameterList, numericalFluentCallback))
		return INFINITE_COST;

	double ret;
	double distance = computeHeadTableDistance();

	// If head to table distance satisfies the given tolerance, then return INFITINITE_COST (= false)
	// because no need to lift torso, otherwise return true
    if (vdist_head_to_table_ - vdist_threshold_ < distance &&
    	distance < vdist_head_to_table_ + vdist_threshold_)
    	ret = INFINITE_COST;
    else
    	ret = 0.0;

    ROS_INFO("lift_torso_modules::%s: Need to lift torso? ", (ret == INFINITE_COST) ? "false" : "true");

	return ret;
}


