#ifndef PLANNING_SCENE_MONITOR_H_
#define PLANNING_SCENE_MONITOR_H_

#include "symbolic_planning_utils/planning_scene_interface.h"
#include "symbolic_planning_utils/planning_scene_monitor_singleton.h"
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
		virtual void publishCollisionObject(const moveit_msgs::CollisionObject& collisionObject);

		private:
		planning_scene_monitor::PlanningSceneMonitorPtr psm_;
		ros::Publisher pubPlanningScene_;

		void getPlanningSceneMsg(moveit_msgs::PlanningScene& psMsg);
	};

};

#endif /* PLANNING_SCENE_MONITOR_H_ */
