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

string create_cache_key(
		const geometry_msgs::Pose& startPose,
		const geometry_msgs::Pose& goalPose);

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

int navigation_effect_grounding(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
		vector<double> & writtenVars);

#ifdef __cplusplus
}
#endif

#endif

