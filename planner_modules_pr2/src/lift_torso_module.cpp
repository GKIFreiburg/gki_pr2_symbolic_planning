#include "planner_modules_pr2/lift_torso_module.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

//#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

VERIFY_CONDITIONCHECKER_DEF(lift_torso_cost);
VERIFY_CONDITIONCHECKER_DEF(need_to_lift_torso);
VERIFY_CONDITIONCHECKER_DEF(is_torso_lifted);
VERIFY_APPLYEFFECT_DEF(update_torso_height);


// ________________________________________________________________________________________________
bool fetch_torso_heights_from_state(modules::numericalFluentCallbackType numericalFluentCallback,
		double& sampled_torso_height, double& current_torso_height)
{
	modules::ParameterList empty;

	// fetch values from planner
	modules::NumericalFluentList nfRequest;
	nfRequest.reserve(2);
	nfRequest.push_back(modules::NumericalFluent("sampled-torso-height", empty));
	nfRequest.push_back(modules::NumericalFluent("current-torso-height", empty));

    modules::NumericalFluentList* nfRequestP = &nfRequest;
    if(!numericalFluentCallback(nfRequestP)) {
        ROS_ERROR("lift_torso_modules::%s: numericalFluentCallback failed.", __func__);
        return false;
    }

    // initialize values
    sampled_torso_height 	= nfRequest[0].value;
    current_torso_height    = nfRequest[1].value;

    ROS_INFO("lift_torso_modules::%s: sampled-torso-height: %lf and current-torso-height: %lf",
    		__func__, sampled_torso_height, current_torso_height);

    return true;
}

// ________________________________________________________________________________________________
void lift_torso_init(int argc, char** argv)
{
	ROS_ASSERT(argc == 1);

    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);

    // /continual_planning_executive/vdist_head_to_table)
    nhPriv.param("/continual_planning_executive/vdist_module_threshold", vdist_threshold_, 0.02);

    // If not defined on param server, take 0.02 m/s which is an arbitrary chosen value
    nhPriv.param("lift_speed", lift_speed_, 0.02);

   	ROS_INFO_STREAM("lift_torso_modules::" << __func__ << ": param namespace: " << nhPriv.getNamespace() << "\n"
		"vdist_module_threshold: " << vdist_threshold_ << "\n"
		"lift_speed: " << lift_speed_);


    ROS_INFO("lift_torso_modules::%s: Initialized lift_torso Module.", __func__);
}

// ________________________________________________________________________________________________
double lift_torso_cost(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	double sampled_torso_height, current_torso_height;

	ROS_WARN("lift_torso_module::%s: followed by fetch_torso_height", __func__);
	if (!fetch_torso_heights_from_state(numericalFluentCallback, sampled_torso_height, current_torso_height))
		return modules::INFINITE_COST;

	double cost = fabs(sampled_torso_height - current_torso_height);

	if (cost < vdist_threshold_)
		cost = vdist_threshold_;

	return cost/lift_speed_;
}

// ________________________________________________________________________________________________
double need_to_lift_torso(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	double sampled_torso_height, current_torso_height;

	ROS_WARN("lift_torso_module::%s: followed by fetch_torso_height", __func__);
	if (!fetch_torso_heights_from_state(numericalFluentCallback, sampled_torso_height, current_torso_height))
	{
		ROS_ERROR("lift_torso_module::%s: STH WENT WRONG", __func__);
		// return 0.0 = true to signal that sth went wrong
		return 0.0;
	}

	if (sampled_torso_height < 0)
	{
		ROS_WARN("lift_torso_module::%s: DID NOT SAMPLED TORSO HEIGHT");
		return 0.0;
	}

	// Taking fabs() of lift distance, since it is not important if going up or down
	double distance = fabs(sampled_torso_height - current_torso_height);

	// If distance satisfies the given tolerance, then return INFITINITE_COST (= false)
	// because no need to lift torso, otherwise return true
	double result;
    if (distance < vdist_threshold_)
    {
    	result = modules::INFINITE_COST;
    	ROS_WARN("lift_torso_module::%s: Do NOT need to lift torso", __func__);
    }
    else
    {
    	result = 0.0;
    	ROS_WARN("lift_torso_module::%s: Need to lift torso", __func__);
    }

    return result;
}

// ________________________________________________________________________________________________
double is_torso_lifted(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	double sampled_torso_height, current_torso_height;

	ROS_WARN("lift_torso_module::%s: followed by fetch_torso_height", __func__);
	if (!fetch_torso_heights_from_state(numericalFluentCallback, sampled_torso_height, current_torso_height))
	{
		ROS_ERROR("lift_torso_module::%s: STH WENT WRONG", __func__);
		// return 0.0 = true to signal that sth went wrong
		return modules::INFINITE_COST;
	}

	if (sampled_torso_height < 0)
	{
		ROS_WARN("lift_torso_module::%s: DID NOT SAMPLED TORSO HEIGHT");
		return modules::INFINITE_COST;
	}

	// Taking fabs() of lift distance, since it is not important if going up or down
	double distance = fabs(sampled_torso_height - current_torso_height);

	// If distance satisfies the given tolerance, then return 0.0 (= true)
	// because torso is already lifted
	double result;
    if (distance < vdist_threshold_)
    {
    	ROS_WARN("lift_torso_module::%s: Torso is lifted", __func__);
    	result = 0.0;
    }
    else
    {
    	ROS_WARN("lift_torso_module::%s: Torso is NOT lifted", __func__);
    	result = modules::INFINITE_COST;
    }

    ROS_WARN("lift_torso_module::%s: RESULT: %lf", __func__, result);
    return result;
}

// ________________________________________________________________________________________________
int update_torso_height(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed, vector<double> & writtenVars)
{
	double sampled_torso_height, current_torso_height;

	if (!fetch_torso_heights_from_state(numericalFluentCallback, sampled_torso_height, current_torso_height))
	{
		ROS_ERROR("lift_torso_module::%s: STH WENT WRONG", __func__);
		// return 0.0 to signal that no variables have been written
		return 0.0;
	}

	ROS_ASSERT(writtenVars.size() == 1);
	writtenVars[0] = sampled_torso_height;
	ROS_WARN("lift_torso_modules::%s: current torso height: %lf", __func__, writtenVars[0]);

	return 1.0; // update state
}
