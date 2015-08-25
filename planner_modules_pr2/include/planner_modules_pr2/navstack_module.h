#ifndef NAVSTACK_MODULE_H
#define NAVSTACK_MODULE_H

#include <tfd_modules/module_api/pddlModuleTypes.h>
#include "planner_modules_pr2/module_param_cache.h"
#include <map>
#include <string>
#include <utility>
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>

using std::string;
/**
 * Simple module implementation for ROS navigation stack.
 *
 * Directly queries the move_base_node/make_plan service for each
 * cost request by the planner.
 */

static const bool g_Debug = false;

extern ros::NodeHandle* g_NodeHandle;
extern ros::ServiceClient g_GetPlan;

/// Plan requests are issued using this frame - so the poses from the planner are given in this frame (e.g. map)
extern std::string g_WorldFrame;

extern double g_GoalTolerance;

/// speed values for computing the estimated time as cost of a path
extern double g_TransSpeed; // m/s
extern double g_RotSpeed;   // rad/s

// Using a cache of queried path costs to prevent calling the path planning service multiple times
// Better: Can we assume symmetric path costs?
//extern std::map< std::pair<std::string, std::string>, double> g_PathCostCache;
//extern ModuleParamCacheDouble g_PathCostCache;
string compute_path_cache_key(const string& startLocation, const string& goalLocation,
        const geometry_msgs::Pose & startPose, const geometry_msgs::Pose & goalPose);

bool fill_path_request(const modules::ParameterList & parameterList, modules::numericalFluentCallbackType numericalFluentCallback,
        nav_msgs::GetPlan::Request& request);

bool fill_robot_pose_XYT(modules::numericalFluentCallbackType numericalFluentCallback, geometry_msgs::PoseStamped& robot_pose);

/// Return the cost of the plan.
/**
 * \param [out] callFailure is set to true, if there was some failure during the call, i.e. the 
 * resulting INFINITE_COST does not represent necessarily the impossibility of a path.
 */
double call_planning_service(nav_msgs::GetPlan& srv, const string& startLocationName, const string& goalLocationName,
        bool & callFailure);

double get_plan_cost(const std::vector<geometry_msgs::PoseStamped> & plan);

#ifdef __cplusplus
extern "C" {
#endif

void navstack_init(int argc, char** argv);

double path_cost(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

double path_cost_grounding(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

double path_condition_grounding(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

int update_robot_pose(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
        vector<double> & writtenVars);

#ifdef __cplusplus
}
#endif

#endif

