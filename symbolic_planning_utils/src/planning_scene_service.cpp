#include "symbolic_planning_utils/planning_scene_service.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>

namespace symbolic_planning_utils
{

	PlanningSceneService::PlanningSceneService()
	{
		ros::NodeHandle nh;
		ROS_INFO("PlanningSceneService::%s: Waiting for GetPlanningScene "
				"service server to start.", __func__);
		getPlanningSceneClient_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
		getPlanningSceneClient_.waitForExistence();

		pubCollisionObject_ = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
		pubAttachedCollisionObject_ = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
	}

	PlanningSceneService::~PlanningSceneService()
	{
	}

	std::vector<moveit_msgs::CollisionObject> PlanningSceneService::getCollisionObjects()
	{
		moveit_msgs::GetPlanningScene::Request request;
		moveit_msgs::GetPlanningScene::Response response;
		request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
		if (!getPlanningSceneClient_.call(request, response))
		{
			ROS_ERROR("PlanningSceneService::%s: planning scene request failed.", __func__);
			return response.scene.world.collision_objects;
		}
		return response.scene.world.collision_objects;
	}

	std::vector<moveit_msgs::AttachedCollisionObject> PlanningSceneService::getAttachedCollisionObjects()
	{
		moveit_msgs::GetPlanningScene::Request request;
		moveit_msgs::GetPlanningScene::Response response;
		request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
		if (!getPlanningSceneClient_.call(request, response))
		{
			ROS_ERROR("PlanningSceneService::%s: planning scene request failed.", __func__);
			return response.scene.robot_state.attached_collision_objects;
		}
		return response.scene.robot_state.attached_collision_objects;
	}

	void PlanningSceneService::publishCollisionObject(const moveit_msgs::CollisionObject& collisionObject)
	{
		pubCollisionObject_.publish(collisionObject);
	}

};
