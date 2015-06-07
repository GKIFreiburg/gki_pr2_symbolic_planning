#ifndef PLANNING_SCENE_MONITOR_SINGLETON_
#define PLANNING_SCENE_MONITOR_SINGLETON_

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

/* THIS IMPLENTATION IS NOT THREAD SAFE */
namespace symbolic_planning_utils
{
	class PlanningSceneMonitorSingleton
	{
	private:
		static PlanningSceneMonitorSingleton* instance_;
		planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

		// Constructor: creating planningSceneMonitor object
		PlanningSceneMonitorSingleton();

		// Copy constructor must be private to prevent any additional creation of the object
		PlanningSceneMonitorSingleton(const PlanningSceneMonitorSingleton* psm);

		// Assignment operator must also be private to prevent any additional creation of the object
		PlanningSceneMonitorSingleton* operator= (const PlanningSceneMonitorSingleton* psm);

		// The destructor is private in order to prevent clients that hold a pointer to the
		// Singleton object from deleting it accidentally.
		// Is not implemented, since the object should exist until the end of execution of the
		// entire program
		virtual ~PlanningSceneMonitorSingleton();

	public:
		static PlanningSceneMonitorSingleton* getInstance();
		planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitorPtr();

	};
};

#endif /* PLANNING_SCENE_MONITOR_SINGLETON_ */
