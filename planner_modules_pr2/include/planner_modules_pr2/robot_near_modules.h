/*
 * robot_near_modules.h
 *
 *  Created on: Aug 27, 2015
 *      Author: andreas
 */

#ifndef ROBOT_NEAR_MODULES_H_
#define ROBOT_NEAR_MODULES_H_

#include "tidyup_planning_scene_updater.h"
#include <tfd_modules/module_api/pddlModuleTypes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <inverse_capability_map/InverseCapabilityOcTree.h>

namespace planner_modules_pr2
{
namespace robot_near_table
{
double compute_value(planning_scene::PlanningScenePtr scene, const string& table, const geometry_msgs::Pose& table_pose);
string create_cache_key(const string& table, const geometry_msgs::Pose2D& robot_pose, double torso_position);
} /* namespace robot_near_table */

} /* namespace planner_modules_pr2 */

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void robot_near_table_init(int argc, char** argv);

// Condition checker Module, check if robot is near table
// INFINITE_COST if false or 0 (a value smaller INFINITE_COST) if true
double robot_near_table(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);

#ifdef __cplusplus
}
#endif // __cplusplus


#endif /* ROBOT_NEAR_MODULES_H_ */
