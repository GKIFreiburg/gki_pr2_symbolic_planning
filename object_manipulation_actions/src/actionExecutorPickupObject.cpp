#include "object_manipulation_actions/actionExecutorPickupObject.h"
#include <pluginlib/class_list_macros.h>

#include <tidyup_utils/stringutil.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <symbolic_planning_utils/planning_scene_monitor.h>
#include <symbolic_planning_utils/planning_scene_service.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_interface/move_group.h>

//PLUGINLIB_DECLARE_CLASS(object_manipulation_actions, action_executor_pickup_object,
//        object_manipulation_actions::ActionExecutorPickupObject,
//        continual_planning_executive::ActionExecutorInterface)
PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorPickupObject, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{

ActionExecutorPickupObject::ActionExecutorPickupObject() :
		actionGenerateGrasps_("generate_grasps", true)
{
	ros::NodeHandle nh;
	ROS_INFO("ActionExecutorPickupObject::%s: Waiting for GetPlanningScene "
			"service server to start.", __func__);
	getPlanningSceneClient_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
	getPlanningSceneClient_.waitForExistence();

	ROS_INFO("ActionExecutorPickupObject::%s: Waiting for generate_grasps "
			"action server to start.", __func__);
	actionGenerateGrasps_.waitForServer(); // will wait for infinite time

	ROS_INFO("ActionExecutorInspectLocation::%s: Action client is ready", __func__);

	psi_.reset(new symbolic_planning_utils::PlanningSceneMonitor());

	// ROS_ERROR("ActionExecutorPickupObject::%s: Loading planning scene monitor.", __func__);
    psm_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
}

ActionExecutorPickupObject::~ActionExecutorPickupObject()
{
}

void ActionExecutorPickupObject::initialize(const std::deque<std::string> & arguments)
{
	ROS_ASSERT(arguments.size() >= 1);
	action_name_ = arguments[0];
}

bool ActionExecutorPickupObject::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
{
	return a.name == action_name_;
}

bool ActionExecutorPickupObject::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
{
	ROS_ASSERT(a.parameters.size() == 4);
	std::string movable_obj = a.parameters[0];
	std::string arm 		= a.parameters[1];
	std::string table 		= a.parameters[2];
	std::string mani_loc 	= a.parameters[3];

	moveit_msgs::GetPlanningScene::Request request;
	moveit_msgs::GetPlanningScene::Response response;
	request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
	if (!getPlanningSceneClient_.call(request, response))
	{
		ROS_ERROR("ActionExecutorPickupObject::%s: planning scene request failed.", __func__);
		return false;
	}

	moveit_msgs::CollisionObject collObj;
	if (!psi_->getObjectFromCollisionObjects(movable_obj, collObj))
	{
		ROS_ERROR("ActionExecutorPickupObject::%s: No collision object found with name %s",
				__func__, movable_obj.c_str());
		return false;
	}

	grasp_provider_msgs::GenerateGraspsGoal goal;
	goal.collision_object = collObj;
	std::string eef_name;
	moveit::planning_interface::MoveGroup* arm_group;
	if (StringUtil::startsWith(arm, "left_"))
	{
		eef_name = "left_gripper";
		arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
	}
	else if (StringUtil::startsWith(arm, "right_"))
	{
		eef_name = "right_gripper";
		arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
	}
	else
	{
		ROS_ERROR("ActionExecutorPickupObject::%s: No arm group could be specified.", __func__);
		return false;
	}





//    // update manually, don't start monitors that continuously update
//    psm_->requestPlanningSceneState();
//    planning_scene_monitor::LockedPlanningSceneRO ps(psm_);
//
//    collision_detection::AllowedCollisionMatrix acm = ps->getAllowedCollisionMatrix();
//    robot_state::RobotState copied_state = ps->getCurrentState();
//
//    collision_detection::CollisionRequest collision_request;
//    collision_detection::CollisionResult collision_result;
//    collision_request.contacts = true;
//    collision_request.max_contacts = 10000;
//    collision_result.clear();
////    ps->checkSelfCollision(collision_request, collision_result);
//    ps->checkCollision(collision_request, collision_result);
//    ROS_WARN_STREAM("ActionExecutorPickupObject::" << __func__ << ": Test 1: Current state is "
//                    << (collision_result.collision ? "in" : "not in")
//                    << " self collision");
//    collision_detection::CollisionResult::ContactMap::const_iterator it2;
//    for(it2 = collision_result.contacts.begin();
//        it2 != collision_result.contacts.end();
//        ++it2)
//    {
//    	ROS_INFO("ActionExecutorPickupObject::%s: Collision found between %s and %s", __func__,
//    			it2->first.first.c_str(), it2->first.second.c_str());
//    	std::string link1 = it2->first.first;
//    	std::string link2 = it2->first.second;
////    	if (StringUtil::startsWith(link1, "r_") || StringUtil::startsWith(link1, "l_") ||
////    		StringUtil::startsWith(link2, "r_") || StringUtil::startsWith(link2, "l_"))
//    	{
//        	ROS_WARN("ActionExecutorPickupObject::%s: Allow collision between %s and %s", __func__,
//        			link1.c_str(), link2.c_str());
//			acm.setEntry(link1, link2, true);
//			acm.setEntry(table, link1, true);
//			acm.setEntry(table, link2, true);
//    	}
//    }
//
//    collision_result.clear();
////    ps->checkSelfCollision(collision_request, collision_result);
//    ps->checkCollision(collision_request, collision_result);
//    ROS_WARN_STREAM("ActionExecutorPickupObject::" << __func__ << ": Test 2: Current state is "
//                    << (collision_result.collision ? "in" : "not in")
//                    << " self collision");
//
//
//	moveit_msgs::PlanningScene psMsg;
//	ps->getPlanningSceneMsg(psMsg);
//
//    psMsg.is_diff = true;  // this is fine, we don't leave anything unspecified that we don't want
//    moveit_msgs::AllowedCollisionMatrix acmMsg;
//    acm.getMessage(acmMsg);
//    psMsg.allowed_collision_matrix = acmMsg;
//
//    ros::NodeHandle nh;
//    ros::Publisher pubPlanningScene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
//    pubPlanningScene.publish(psMsg);
//    ros::Duration(2.0).sleep();












	arm_group->getCurrentState();
	goal.eef_group_name = eef_name;
	actionGenerateGrasps_.sendGoal(goal);

	// wait for the action to return
	bool finished_before_timeout = actionGenerateGrasps_.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionGenerateGrasps_.getState();
		ROS_INFO("ActionExecutorPickupObject::%s: Generate grasp action %s.", __func__, state.toString().c_str());
		if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("ActionExecutorPickupObject::%s: Generate grasp action failed.", __func__);
			return false;
		}
		std::vector<moveit_msgs::Grasp> grasps = actionGenerateGrasps_.getResult()->grasps;
		ROS_DEBUG("ActionExecutorPickupObject::%s: %lu grasps were found.", __func__, grasps.size());
		moveit::planning_interface::MoveItErrorCode error_code;
		arm_group->setSupportSurfaceName(table);
		error_code = arm_group->pick(collObj.id, grasps);
		ROS_INFO_STREAM("ActionExecutorPickupObject::" << __func__ <<": Pickup object " << collObj.id << " action returned "
				<< error_code);
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	} else {
		ROS_ERROR("ActionExecutorPickupObject: Generate grasp action did not finish before the time out.");
		return false;
	}
	return true;
}

void ActionExecutorPickupObject::cancelAction()
{
}

}; // namespace
