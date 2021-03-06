#include <pluginlib/class_list_macros.h>

#include <tidyup_utils/stringutil.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <symbolic_planning_utils/planning_scene_monitor.h>
#include <symbolic_planning_utils/planning_scene_service.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_interface/move_group.h>
#include <object_manipulation_actions/PickupObject.h>

//PLUGINLIB_DECLARE_CLASS(object_manipulation_actions, action_executor_pickup_object,
//        object_manipulation_actions::ActionExecutorPickupObject,
//        continual_planning_executive::ActionExecutorInterface)
PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::PickupObject,
		continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{

PickupObject::PickupObject() :
		actionGenerateGrasps_("generate_grasps", true)
{
	ros::NodeHandle nh;
	ROS_INFO_STREAM(action_name_<<": Waiting for GetPlanningScene "
			"service server to start.");
	getPlanningSceneClient_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(
			move_group::GET_PLANNING_SCENE_SERVICE_NAME);
	getPlanningSceneClient_.waitForExistence();

	ROS_INFO_STREAM(action_name_<<": Waiting for generate_grasps "
			"action server to start.");
	actionGenerateGrasps_.waitForServer(); // will wait for infinite time

	ROS_INFO_STREAM(action_name_<<": Action client is ready");

	// better use service calls rather than planningSceneMontior,
	// which needs the initialize the robot description - takes time)
	// psi_.reset(new symbolic_planning_utils::PlanningSceneMonitor());
	psi_.reset(new symbolic_planning_utils::PlanningSceneService());
}

PickupObject::~PickupObject()
{
}

void PickupObject::initialize(const std::deque<std::string> & arguments)
{
	ROS_ASSERT(arguments.size() >= 2);
	action_name_ = arguments[0];
	predicate_inspected_recently_ = arguments[1];
}

bool PickupObject::canExecute(const DurativeAction & a,
		const SymbolicState & currentState) const
{
	return a.name == action_name_;
}

bool PickupObject::executeBlocking(const DurativeAction & a,
		SymbolicState & currentState)
{
	ROS_ASSERT(a.parameters.size() == 3);
	std::string movable_obj = a.parameters[0];
	std::string arm = a.parameters[1];
	std::string table = a.parameters[2];

	moveit_msgs::GetPlanningScene::Request request;
	moveit_msgs::GetPlanningScene::Response response;
	request.components.components =
			moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
	if (!getPlanningSceneClient_.call(request, response))
	{
		ROS_ERROR_STREAM(action_name_<<": planning scene request failed.");
		return false;
	}

	moveit_msgs::CollisionObject collObj;
	if (!psi_->getObjectFromCollisionObjects(movable_obj, collObj))
	{
		ROS_ERROR_STREAM(action_name_<<": No collision object found with name "<<movable_obj);
		return false;
	}

	grasp_provider_msgs::GenerateGraspsGoal goal;
	goal.collision_object = collObj;
	std::string eef_name;
	moveit::planning_interface::MoveGroup* arm_group;
	if (StringUtil::startsWith(arm, "left_"))
	{
		eef_name = "left_gripper";
		arm_group =
				symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
	}
	else if (StringUtil::startsWith(arm, "right_"))
	{
		eef_name = "right_gripper";
		arm_group =
				symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
	}
	else
	{
		ROS_ERROR_STREAM(action_name_<<": arm group lookup failed. expected right_arm or left_arm; got "<<a.parameters[0]);
		return false;
	}

	arm_group->getCurrentState();
	goal.eef_group_name = eef_name;
	actionGenerateGrasps_.sendGoal(goal);

	// wait for the action to return
	bool finished_before_timeout = actionGenerateGrasps_.waitForResult(
			ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionGenerateGrasps_.getState();
		ROS_INFO_STREAM(action_name_<<": Generate grasp action "<<state.toString());
		if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR_STREAM(action_name_<<": grasp generation failed");
			return false;
		}
		std::vector<moveit_msgs::Grasp> grasps =
				actionGenerateGrasps_.getResult()->grasps;
		ROS_DEBUG_STREAM(action_name_<<": "<<grasps.size()<<" grasps were found.");
		moveit::planning_interface::MoveItErrorCode error_code;
		arm_group->setSupportSurfaceName(table);
		error_code = arm_group->pick(collObj.id, grasps);
		ROS_INFO_STREAM(action_name_<<": pickup object " << collObj.id << " action failed: " << error_code);

		// set predicate *-inspected-recently to false after successful grasp
		currentState.setBooleanPredicate(predicate_inspected_recently_, table,
				false);

		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}
	else
	{
		ROS_ERROR_STREAM(action_name_<<": time out during grasp generation");
		return false;
	}

}

void PickupObject::cancelAction()
{
}

}
;
// namespace
