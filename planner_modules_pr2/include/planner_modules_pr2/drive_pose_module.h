#ifndef DRIVE_POSE_MODULE_H
#define DRIVE_POSE_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <ros/ros.h>

const double INFINITE_COST = HUGE_VAL;

std::string world_frame_;

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
