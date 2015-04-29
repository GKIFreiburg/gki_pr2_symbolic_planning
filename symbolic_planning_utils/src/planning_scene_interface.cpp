#include "symbolic_planning_utils/planning_scene_interface.h"

#include <ros/ros.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

namespace symbolic_planning_utils
{

PlanningSceneInterface::PlanningSceneInterface()
{
}

PlanningSceneInterface::~PlanningSceneInterface()
{
}

bool PlanningSceneInterface::getObjectFromCollisionObjects(const std::string& objectName, moveit_msgs::CollisionObject& collisionObject)
{
    std::vector<moveit_msgs::CollisionObject> cos = getCollisionObjects();
    if (cos.size() == 0)
    {
    	ROS_WARN("PlanningSceneInterface::%s: No collision objects in vector.", __func__);
    	return false;
    }
    forEach(const moveit_msgs::CollisionObject & co, cos) {
        if(co.id == objectName) {
            collisionObject = co;
            return true;
        }
    }
    return false;
}

bool PlanningSceneInterface::getAttachedObjectFromAttachedCollisionObjects(const std::string& objectName,
		moveit_msgs::AttachedCollisionObject& attachedCollisionObject)
{
    std::vector<moveit_msgs::AttachedCollisionObject> acos = getAttachedCollisionObjects();
    if (acos.size() == 0)
    {
    	ROS_WARN("PlanningSceneInterface::%s: No attached collision objects in vector.", __func__);
    	return false;
    }
    forEach(const moveit_msgs::AttachedCollisionObject & aco, acos) {
        if(aco.object.id == objectName) {
        	attachedCollisionObject = aco;
            return true;
        }
    }
    return false;
}

};
