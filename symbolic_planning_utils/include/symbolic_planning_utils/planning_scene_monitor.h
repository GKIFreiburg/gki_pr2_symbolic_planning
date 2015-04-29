#ifndef PLANNING_SCENE_MONITOR_H_
#define PLANNING_SCENE_MONITOR_H_

#include "planning_scene_interface.h"
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace symbolic_planning_utils
{
	class PlanningSceneMonitor : public PlanningSceneInterface
	{
		public:

		PlanningSceneMonitor();
		virtual ~PlanningSceneMonitor();

		virtual std::vector<moveit_msgs::CollisionObject> getCollisionObjects();
		virtual std::vector<moveit_msgs::AttachedCollisionObject> getAttachedCollisionObjects();

		private:
		boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
	};

};

#endif /* PLANNING_SCENE_MONITOR_H_ */
