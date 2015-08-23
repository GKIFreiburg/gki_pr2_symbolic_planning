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

//double table_height_;
//double torso_position_;
double vdist_head_to_table_;
double vdist_threshold_;
double lift_speed_;

// set the variable table_height_ and torso_position_
bool fetch_variables_from_planner(const modules::ParameterList & parameterList,
		modules::numericalFluentCallbackType numericalFluentCallback,
		double& table_height, double& torso_position);

// Compute the real distance that torso needs to be lifted, considering the joint limits of
// torso_lift_link. If result is positiv, meaning torso is going to be raised, if otherwise
// torso is lowered
double compute_lift_distance(const double& table_height, const double& torso_position);



#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void lift_torso_init(int argc, char** argv);

// Cost Module, computing real costs for lifting torso
double lift_torso_cost(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);

// Condition checker Module, check if torso needs to be lifted
// INFINITE_COST if false or 0 (a value smaller INFINITE_COST) if true
double need_to_lift_torso(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);

// Condition checker Module, check if torso is lifted
// INFINITE_COST if false or 0 (a value smaller INFINITE_COST) if true
// Inverse of needToLiftTorso
double torso_lifted(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);

// Effect Module, set predicate "torso-lifted" with the corresponding table
// If return value != 0, then writtenVars are applied to the state, otherwise state remains unchanged
int update_torso_position(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed, vector<double> & writtenVars);

#ifdef __cplusplus
}
#endif // __cplusplus


#endif // LIFT_TORSO_MODULE_H
