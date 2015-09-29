#ifndef DRIVE_POSE_MODULE_H
#define DRIVE_POSE_MODULE_H

#include <tfd_modules/module_api/pddlModuleTypes.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <inverse_capability_map/InverseCapabilityOcTree.h>

namespace planner_modules_pr2
{

namespace drive_pose
{

// Look up surface id in cache and return corresponding pose
bool lookup_pose_from_surface_id(const std::string& surface,
		geometry_msgs::PoseStamped& pose);

// Push drive_pose_cache on param
void set_poses_on_param(const std::string& name_space,
		const std::map<std::string, geometry_msgs::PoseStamped>& drive_poses);

// Fetch drive poses from param and store in cache again
void fetch_poses_from_param(const std::string& name_space, const std::string& surface,
		std::map<std::string, geometry_msgs::PoseStamped>& drive_poses);

} /* namespace drive_pose */

} /* namespace planner_modules_pr2 */

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void drive_pose_init(int argc, char** argv);

void drive_pose_exit(const modules::RawPlan & plan, int argc, char** argv,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback);

/// Determine a candidate pose to drive to the static ?t - table.
std::string determine_drive_pose(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback, modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed, const void* statePtr);

#ifdef __cplusplus
}
#endif // __cplusplus


#endif // DRIVE_POSE_MODULE_H
