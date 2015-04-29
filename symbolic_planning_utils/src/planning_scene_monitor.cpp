#include "symbolic_planning_utils/planning_scene_monitor.h"

namespace symbolic_planning_utils
{

	PlanningSceneMonitor::PlanningSceneMonitor()
	{
	    psm_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
	}

	PlanningSceneMonitor::~PlanningSceneMonitor()
	{
	}

	std::vector<moveit_msgs::CollisionObject> PlanningSceneMonitor::getCollisionObjects()
	{
	    // update manually, don't start monitors that continuously update
	    psm_->requestPlanningSceneState();
	    planning_scene_monitor::LockedPlanningSceneRO ps(psm_);

	    moveit_msgs::PlanningScene psMsg;
	    ps->getPlanningSceneMsg(psMsg);

	    return psMsg.world.collision_objects;
	}

	std::vector<moveit_msgs::AttachedCollisionObject> PlanningSceneMonitor::getAttachedCollisionObjects()
	{
	    // update manually, don't start monitors that continuously update
	    psm_->requestPlanningSceneState();
	    planning_scene_monitor::LockedPlanningSceneRO ps(psm_);

	    moveit_msgs::PlanningScene psMsg;
	    ps->getPlanningSceneMsg(psMsg);

	    return psMsg.robot_state.attached_collision_objects;
	}

};
