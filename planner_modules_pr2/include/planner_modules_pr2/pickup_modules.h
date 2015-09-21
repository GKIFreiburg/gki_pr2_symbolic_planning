/*
 * PickupModules.h
 *
 *  Created on: Aug 20, 2015
 *      Author: andreas
 */

#ifndef PICKUPMODULES_H_
#define PICKUPMODULES_H_
#include "planner_modules_pr2/manipulation_planning.h"

#include <tfd_modules/module_api/pddlModuleTypes.h>
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include "planner_modules_pr2/module_param_cache.h"

#include <string>
#include <ros/ros.h>

namespace planner_modules_pr2
{
namespace pickup
{

double compute_value(
		planning_scene::PlanningScenePtr scene,
		const string& object_name,
		const string& arm_prefix,
		const string& table_name);

string create_cache_key(const string& object,
		const string& arm,
		const string& table,
		const geometry_msgs::Pose2D& robot_pose,
		double torso_position,
		const map<string, geometry_msgs::Pose>& movableObjects,
		const map<string, string>& objectsOnStatic);

}/* pickup */
} /* namespace planner_modules_pr2 */

#ifdef __cplusplus
extern "C" {
#endif

void pickup_init(int argc, char** argv);

double can_pickup(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

double can_pickup_grounding(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

int pickup_effect(const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
		vector<double> & writtenVars);

int pickup_effect_grounding(const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
		vector<double> & writtenVars);

#ifdef __cplusplus
}
#endif

#endif /* PICKUPMODULES_H_ */
