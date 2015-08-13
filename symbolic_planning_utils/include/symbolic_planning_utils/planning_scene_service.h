#ifndef PLANNING_SCENE_SERVICE_H_
#define PLANNING_SCENE_SERVICE_H_

#include "symbolic_planning_utils/planning_scene_interface.h"
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <ros/service.h>
#include <ros/publisher.h>

namespace symbolic_planning_utils
{
	class PlanningSceneService : public PlanningSceneInterface
	{
		public:

		PlanningSceneService();
		virtual ~PlanningSceneService();

		virtual std::vector<moveit_msgs::CollisionObject> getCollisionObjects();
		virtual std::vector<moveit_msgs::AttachedCollisionObject> getAttachedCollisionObjects();

		virtual void publishCollisionObject(const moveit_msgs::CollisionObject& collisionObject);

		private:
		ros::ServiceClient getPlanningSceneClient_;
		ros::Publisher pubCollisionObject_;
		ros::Publisher pubAttachedCollisionObject_;
	};

};

#endif /* PLANNING_SCENE_SERVICE_H_ */
