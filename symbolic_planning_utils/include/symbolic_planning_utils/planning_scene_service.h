#ifndef PLANNING_SCENE_SERVICE_H_
#define PLANNING_SCENE_SERVICE_H_

#include "planning_scene_interface.h"
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>


namespace symbolic_planning_utils
{
	class PlanningSceneService : public PlanningSceneInterface
	{
		public:

		PlanningSceneService();
		virtual ~PlanningSceneService();

		virtual std::vector<moveit_msgs::CollisionObject> getCollisionObjects();
		virtual std::vector<moveit_msgs::AttachedCollisionObject> getAttachedCollisionObjects();

		private:
		ros::ServiceClient getPlanningSceneClient_;
	};

};

#endif /* PLANNING_SCENE_SERVICE_H_ */
