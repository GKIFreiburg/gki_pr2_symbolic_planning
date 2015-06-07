#include "symbolic_planning_utils/planning_scene_monitor_singleton.h"

namespace symbolic_planning_utils
{

PlanningSceneMonitorSingleton* PlanningSceneMonitorSingleton::instance_ = NULL;

PlanningSceneMonitorSingleton::PlanningSceneMonitorSingleton()
{
	planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
}

PlanningSceneMonitorSingleton::PlanningSceneMonitorSingleton(const PlanningSceneMonitorSingleton* psm)
{
	PlanningSceneMonitorSingleton::instance_ = psm->instance_;
}

PlanningSceneMonitorSingleton* PlanningSceneMonitorSingleton::operator= (const PlanningSceneMonitorSingleton* psm)
{
	if (this != psm)
	{
		PlanningSceneMonitorSingleton::instance_ = psm->instance_;
	}
	return this;
}

PlanningSceneMonitorSingleton::~PlanningSceneMonitorSingleton()
{
}

PlanningSceneMonitorSingleton* PlanningSceneMonitorSingleton::getInstance()
{
	if (PlanningSceneMonitorSingleton::instance_ == NULL)
		PlanningSceneMonitorSingleton::instance_ = new PlanningSceneMonitorSingleton();
	return PlanningSceneMonitorSingleton::instance_;
}

planning_scene_monitor::PlanningSceneMonitorPtr PlanningSceneMonitorSingleton::getPlanningSceneMonitorPtr()
{
	return planning_scene_monitor_;
}

};
