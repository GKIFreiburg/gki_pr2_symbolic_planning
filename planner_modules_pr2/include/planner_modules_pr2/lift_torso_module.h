#ifndef LIFT_TORSO_MODULE_H
#define LIFT_TORSO_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <ros/ros.h>

// Distance measured from ground when torso is at minimum (= not lifted)
const double MIN_TORSO_POSITION = 0.802;
// Offset between torso_lift_link and head_pan_link (from tf)
const double OFFSET_TORSO_HEAD = 0.38145;
// Soft Joint limits for torso_lift_link (from urdf file)
const double MIN_TORSO_JOINT = 0.00115;
const double MAX_TORSO_JOINT = 0.325;

const double INFINITE_COST = HUGE_VAL;


double table_height_;
double torso_position_;
double vdist_head_to_table_;
double vdist_threshold_;
double lift_speed_;
std::string world_frame_;

// set the variable table_height_ and torso_position_
bool fetchVariablesFromPlanner(const modules::ParameterList & parameterList,
		modules::numericalFluentCallbackType numericalFluentCallback);

// Compute distance between head and table
double computeHeadTableDistance();

// Compute the real distance that torso needs to be lifted, considering the joint limits of
// torso_lift_link
double computeLiftDistance();



#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void liftTorsoInit(int argc, char** argv);

// Cost Module, computing real costs for lifting torso
double liftTorsoCost(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);

// Condition checker Module, check if torso needs to be lifted
// INFINITE_COST if false or 0 (a value smaller INFINITE_COST) if true
// INFINITE_COST = HUGE_VAL
double needToLiftTorso(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);

#ifdef __cplusplus
}
#endif // __cplusplus


#endif // LIFT_TORSO_MODULE_H

