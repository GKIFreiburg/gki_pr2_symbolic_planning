#ifndef PLANNING_SCENE_INTERFACE_H_
#define PLANNING_SCENE_INTERFACE_H_

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

namespace symbolic_planning_utils
{
	class PlanningSceneInterface
	{
		public:

		PlanningSceneInterface();
		virtual ~PlanningSceneInterface();

		virtual std::vector<moveit_msgs::CollisionObject> getCollisionObjects() = 0;
		virtual std::vector<moveit_msgs::AttachedCollisionObject> getAttachedCollisionObjects() = 0;

		bool getObjectFromCollisionObjects(const std::string& objectName, moveit_msgs::CollisionObject& collisionObject);
		// Change id of collision object and update it in the planning scene
		void renameCollisionObject(const std::string& newName, moveit_msgs::CollisionObject& collisionObject);

		bool getAttachedObjectFromAttachedCollisionObjects(const std::string& objectName, moveit_msgs::AttachedCollisionObject& attachedCollisionObject);

		protected:
		virtual void publishCollisionObject(const moveit_msgs::CollisionObject& collisionObject) = 0;

	};
};


#endif /* PLANNINGSCENEINTERFACE_H_ */
