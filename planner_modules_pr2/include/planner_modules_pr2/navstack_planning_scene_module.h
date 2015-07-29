#ifndef NAVSTACK_PLANNING_SCENE_MODULE_H
#define NAVSTACK_PLANNING_SCENE_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include "tidyup_utils/arm_state.h"
//#include "tidyup_utils/planning_scene_interface.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace modules;

#ifdef __cplusplus
extern "C" {
#endif

void planning_scene_navstack_init(int argc, char** argv);

void planning_scene_navstack_exit(const RawPlan & plan, int argc, char** argv,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback);


double planning_scene_pathCost(const ParameterList & parameterList, predicateCallbackType predicateCallback,
      numericalFluentCallbackType numericalFluentCallback, int relaxed);

#ifdef __cplusplus
}
#endif


#endif

