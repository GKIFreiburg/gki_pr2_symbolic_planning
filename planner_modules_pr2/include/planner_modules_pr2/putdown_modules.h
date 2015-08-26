/*
 * putdownModules.h
 *
 *  Created on: Aug 20, 2015
 *      Author: andreas
 */

#ifndef PUTDOWNMODULES_H_
#define PUTDOWNMODULES_H_
#include "planner_modules_pr2/manipulation_planning.h"

#include <tfd_modules/module_api/pddlModuleTypes.h>
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include "planner_modules_pr2/module_param_cache.h"

#include <string>
#include <ros/ros.h>

double putdown(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

std::string compute_putdown_cache_key(
		const string& object_name,
		const string& arm_name,
		const string& table_name,
		const geometry_msgs::Pose2D& robot_pose,
		const map<string, geometry_msgs::Pose>& movableObjects,
		const ObjectsOnTablesMap& objectsOnTables);

#ifdef __cplusplus
extern "C" {
#endif

void putdown_init(int argc, char** argv);

double putdown_cost(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

double can_putdown(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed);

#ifdef __cplusplus
}
#endif

#endif /* PUTDOWNMODULES_H_ */
