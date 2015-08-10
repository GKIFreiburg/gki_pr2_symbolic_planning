#ifndef DRIVE_POSE_MODULE_H
#define DRIVE_POSE_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <inverse_capability_map/InverseCapabilityOcTree.h>

#include <actionlib/client/simple_action_client.h>
#include <planner_modules_pr2/EmptyAction.h>

// debug tools, g_actionDebug enable the triggering of the published msgs
boost::shared_ptr<actionlib::SimpleActionClient<planner_modules_pr2::EmptyAction> > g_action_debug;
// publish the planning scene msgs
extern ros::Publisher g_debug_ps_pub;

const double INFINITE_COST = HUGE_VAL;

// storing the table names with their poses
extern std::map<std::string, geometry_msgs::PoseStamped> g_table_poses;
// storing the table names with their inverse reachability map
extern std::map<std::string, InverseCapabilityOcTree*> g_inv_reach_maps;

extern int g_inv_reach_sample_draws;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void drivePoseInit(int argc, char** argv);

//// Cost Module, computing real costs for lifting torso
//double liftTorsoCost(const modules::ParameterList & parameterList,
//		modules::predicateCallbackType predicateCallback,
//		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);
//
//// Condition checker Module, check if torso needs to be lifted
//// INFINITE_COST if false or 0 (a value smaller INFINITE_COST) if true
//double needToLiftTorso(const modules::ParameterList & parameterList,
//		modules::predicateCallbackType predicateCallback,
//		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);
//
//// Condition checker Module, check if torso is lifted
//// INFINITE_COST if false or 0 (a value smaller INFINITE_COST) if true
//// Inverse of needToLiftTorso
//double torsoLifted(const modules::ParameterList & parameterList,
//		modules::predicateCallbackType predicateCallback,
//		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);
//
//// Effect Module, set predicate "torso-lifted" with the corresponding table
//// If return value != 0, then writtenVars are applied to the state, otherwise state remains unchanged
//int updateTorsoPosition(const modules::ParameterList & parameterList,
//        modules::predicateCallbackType predicateCallback,
//        modules::numericalFluentCallbackType numericalFluentCallback,
//        int relaxed, vector<double> & writtenVars);

/// Determine a candidate pose to drive to the static ?s.
std::string determineDrivePose(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback, modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed, const void* statePtr);

#ifdef __cplusplus
}
#endif // __cplusplus


#endif // DRIVE_POSE_MODULE_H
