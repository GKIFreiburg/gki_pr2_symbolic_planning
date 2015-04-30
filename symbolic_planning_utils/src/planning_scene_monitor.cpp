#include "symbolic_planning_utils/planning_scene_monitor.h"

namespace symbolic_planning_utils
{

	PlanningSceneMonitor::PlanningSceneMonitor()
	{
	    psm_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

	    ros::NodeHandle nh;
	    pubPlanningScene_ = pubPlanningScene_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	}

	PlanningSceneMonitor::~PlanningSceneMonitor()
	{
	}

	void PlanningSceneMonitor::getPlanningSceneMsg(moveit_msgs::PlanningScene& psMsg)
	{
	    // update manually, don't start monitors that continuously update
	    psm_->requestPlanningSceneState();
	    planning_scene_monitor::LockedPlanningSceneRO ps(psm_);

	    ps->getPlanningSceneMsg(psMsg);

	    return;
	}

	std::vector<moveit_msgs::CollisionObject> PlanningSceneMonitor::getCollisionObjects()
	{
		moveit_msgs::PlanningScene psMsg;
		getPlanningSceneMsg(psMsg);

	    return psMsg.world.collision_objects;
	}

	std::vector<moveit_msgs::AttachedCollisionObject> PlanningSceneMonitor::getAttachedCollisionObjects()
	{
		moveit_msgs::PlanningScene psMsg;
		getPlanningSceneMsg(psMsg);

	    return psMsg.robot_state.attached_collision_objects;
	}

	void PlanningSceneMonitor::publishCollisionObject(const moveit_msgs::CollisionObject& collisionObject)
	{
		moveit_msgs::PlanningScene psMsg;
		getPlanningSceneMsg(psMsg);

	    psMsg.is_diff = true;  // this is fine, we don't leave anything unspecified that we don't want
	    psMsg.world.collision_objects.push_back(collisionObject);

	    // Publish Message
	    pubPlanningScene_.publish(psMsg);
	}

};
