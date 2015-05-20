#ifndef LIFT_TORSO_MODULE_H
#define LIFT_TORSO_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <ros/ros.h>


#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void lift_torso_init(int argc, char** argv);

double lift_torso_cost(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed);

#ifdef __cplusplus
}
#endif // __cplusplus


#endif // LIFT_TORSO_MODULE_H

