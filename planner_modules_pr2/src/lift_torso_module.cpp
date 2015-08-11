#include "planner_modules_pr2/lift_torso_module.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

//#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

VERIFY_CONDITIONCHECKER_DEF(liftTorsoCost);
VERIFY_CONDITIONCHECKER_DEF(needToLiftTorso);
VERIFY_CONDITIONCHECKER_DEF(torsoLifted);
VERIFY_APPLYEFFECT_DEF(updateTorsoPosition);


// ________________________________________________________________________________________________
bool fetchVariablesFromPlanner(const modules::ParameterList & parameterList,
		modules::numericalFluentCallbackType numericalFluentCallback,
		double& table_height,
		double& torso_position)
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
    table_height 	= nfRequest[0].value;
    torso_position  = nfRequest[1].value;

//    ROS_INFO("lift_torso_modules::%s: table height: %lf and torso_position: %lf",
//    		__func__, table_height, torso_position);

    return true;
}

// ________________________________________________________________________________________________
double computeLiftDistance(const double& table_height, const double& torso_position)
{

	double head_height = MIN_TORSO_POSITION + torso_position + OFFSET_TORSO_HEAD;
//	ROS_WARN("%s: head_height = %lf + %lf + %lf = %lf", __func__, MIN_TORSO_POSITION,
//			torso_position, OFFSET_TORSO_HEAD, head_height);
	double head_table_dist = head_height - table_height;
//	ROS_WARN("%s: head_table_dist = %lf - %lf = %lf", __func__, head_height, table_height,
//			head_table_dist);

	// Compute height that torso needs to be lifted
	double real_distance = vdist_head_to_table_ - head_table_dist;
//	ROS_WARN("%s: real_distance = %lf - %lf = %lf", __func__, vdist_head_to_table_,
//			head_table_dist, real_distance);

	double new_position = torso_position + real_distance;
//	ROS_WARN("%s: new_position = %lf + %lf = %lf", __func__, torso_position, real_distance, new_position);
	if (new_position < MIN_TORSO_JOINT)
		new_position = MIN_TORSO_JOINT;
	else if (new_position > MAX_TORSO_JOINT)
		new_position = MAX_TORSO_JOINT;

	double lift_distance = new_position - torso_position;
//	ROS_WARN("%s: lift_distance = %lf - %lf = %lf", new_position, torso_position, lift_distance);

//	ROS_INFO("lift_torso_modules::%s: Real distance: %lf, lift distance: %lf", __func__,
//			real_distance, lift_distance);

	return lift_distance;
}

// ________________________________________________________________________________________________
void liftTorsoInit(int argc, char** argv)
{
	ROS_ASSERT(argc == 1);

    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);

    // /continual_planning_executive/vdist_head_to_table)
    nhPriv.param("/continual_planning_executive/vdist_head_to_table", vdist_head_to_table_, 0.60);
    nhPriv.param("/continual_planning_executive/vdist_module_threshold", vdist_threshold_, 0.02);

    // If not defined on param server, take 0.02 m/s which is an arbitrary chosen value
    nhPriv.param("lift_speed", lift_speed_, 0.02);

   	ROS_INFO_STREAM("lift_torso_modules::" << __func__ << ": param namespace: " << nhPriv.getNamespace() << "\n"
		"vdist_head_to_table: " << vdist_head_to_table_ << "\n"
		"vdist_module_threshold: " << vdist_threshold_ << "\n"
		"lift_speed: " << lift_speed_);


    ROS_INFO("lift_torso_modules::%s: Initialized lift_torso Module.", __func__);
}

// ________________________________________________________________________________________________
double liftTorsoCost(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	double table_height, torso_position;

	if (!fetchVariablesFromPlanner(parameterList, numericalFluentCallback, table_height, torso_position))
		return modules::INFINITE_COST;

	double cost = fabs(computeLiftDistance(table_height, torso_position));

	if (cost < vdist_threshold_)
		cost = vdist_threshold_;

	return cost/lift_speed_;
}

// ________________________________________________________________________________________________
double needToLiftTorso(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	double table_height, torso_position;

	if (!fetchVariablesFromPlanner(parameterList, numericalFluentCallback, table_height, torso_position))
		return modules::INFINITE_COST;

	// Taking fabs() of lift distance, since it is not important if going up or down
	double distance = fabs(computeLiftDistance(table_height, torso_position));

//	ROS_WARN("%s: check if: distance < threshold: %lf < %lf", __func__, distance, vdist_threshold_);

	// If head to table distance satisfies the given tolerance, then return INFITINITE_COST (= false)
	// because no need to lift torso, otherwise return true
    if (distance < vdist_threshold_)
    {
    	return modules::INFINITE_COST;
    }
    else
    {
//    	ROS_INFO("lift_torso_modules::%s: Need to lift torso", __func__);
    	return 0.0;
    }
}

// ________________________________________________________________________________________________
double torsoLifted(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	double table_height, torso_position;

	if (!fetchVariablesFromPlanner(parameterList, numericalFluentCallback, table_height, torso_position))
		return modules::INFINITE_COST;

	// Taking fabs() of lift distance, since it is not important if going up or down
	double distance = fabs(computeLiftDistance(table_height, torso_position));

//	ROS_WARN("%s: check if: distance < threshold: %lf < %lf", __func__, distance, vdist_threshold_);

	// If head to table distance satisfies the given tolerance, then return INFITINITE_COST (= false)
	// because no need to lift torso, otherwise return true
    if (distance < vdist_threshold_)
    {
    	return 0.0;
    }
    else
    {
//    	ROS_INFO("lift_torso_modules::%s: Need to lift torso", __func__);
    	return modules::INFINITE_COST;
    }
}

// ________________________________________________________________________________________________
int updateTorsoPosition(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed, vector<double> & writtenVars)
{
	double table_height, torso_position;

//	ROS_INFO("lift_torso_modules::%s: ", __func__);
	if (!fetchVariablesFromPlanner(parameterList, numericalFluentCallback, table_height, torso_position))
		return 0.0; // state remains unchanged

	double lift_distance = computeLiftDistance(table_height, torso_position);
	double new_position = torso_position + lift_distance;

	// For safety reason verify that new_position is not outside bounds
	if (new_position < MIN_TORSO_JOINT)
	{
		ROS_WARN("lift_torso_module::%s: new_position (%lf) < MIN_TORSO_JOINT!", __func__, new_position);
		new_position = MIN_TORSO_JOINT;
	} else if (new_position > MAX_TORSO_JOINT)
	{
		ROS_WARN("lift_torso_module::%s: new_position (%lf) > MAX_TORSO_JOINT!", __func__, new_position);
		new_position = MAX_TORSO_JOINT;
	}

	ROS_ASSERT(writtenVars.size() == 1);
	writtenVars[0] = new_position;
	ROS_WARN("lift_torso_modules::%s: new torso position: %lf", __func__, writtenVars[0]);

	return 1.0; // update state
}
