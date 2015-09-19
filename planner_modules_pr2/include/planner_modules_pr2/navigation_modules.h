#ifndef NAVSTACK_MODULE_H
#define NAVSTACK_MODULE_H

#include <tfd_modules/module_api/pddlModuleTypes.h>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <moveit/planning_scene/planning_scene.h>

using std::string;
/**
 * Simple module implementation for ROS navigation stack.
 *
 * Directly queries the move_base_node/make_plan service for each
 * cost request by the planner.
 */

namespace planner_modules_pr2
{
namespace navigation
{

//static const bool g_Debug = false;
//
//extern ros::ServiceClient g_GetPlan;
//
///// Plan requests are issued using this frame - so the poses from the planner are given in this frame (e.g. map)
//extern std::string g_WorldFrame;
//
//extern double g_GoalTolerance;

/// speed values for computing the estimated time as cost of a path
//extern double linear_velocity; // m/s
//extern double angular_velocity;   // rad/s

// Using a cache of queried path costs to prevent calling the path planning service multiple times
// Better: Can we assume symmetric path costs?
//extern std::map< std::pair<std::string, std::string>, double> g_PathCostCache;
//extern ModuleParamCacheDouble g_PathCostCache;
string create_cache_key(
		const geometry_msgs::Pose& startPose,
		const geometry_msgs::Pose& goalPose);

//bool fill_path_request(const modules::ParameterList & parameterList, modules::numericalFluentCallbackType numericalFluentCallback,
//        nav_msgs::GetPlan::Request& request);

/// Return the cost of the plan.
/**
 * \param [out] callFailure is set to true, if there was some failure during the call, i.e. the 
 * resulting INFINITE_COST does not represent necessarily the impossibility of a path.
 */
//double call_planning_service(nav_msgs::GetPlan& srv, const string& startLocationName, const string& goalLocationName,
//        bool & callFailure);

double compute_value(planning_scene::PlanningScenePtr scene, nav_msgs::GetPlan& srv);

double get_plan_cost(const std::vector<geometry_msgs::PoseStamped> & plan);

} /* namespace planner_modules_pr2 */

} /* namespace navigation */

#ifdef __cplusplus
extern "C" {
#endif

void navigation_init(int argc, char** argv);

double navigation_cost(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

double navigation_cost_grounding(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

int navigation_effect(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
		vector<double> & writtenVars);

#ifdef __cplusplus
}
#endif

#endif

