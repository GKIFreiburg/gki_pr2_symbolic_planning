#include <pluginlib/class_list_macros.h>

#include <tidyup_utils/stringutil.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <symbolic_planning_utils/planning_scene_monitor.h>
#include <symbolic_planning_utils/planning_scene_service.h>
#include <object_surface_placements/placement_generator_discretization.h>
#include <object_surface_placements/placement_generator_sampling.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <object_manipulation_actions/PutdownObject.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

#include <moveit_msgs/MoveItErrorCodes.h>

//PLUGINLIB_DECLARE_CLASS(object_manipulation_actions, action_executor_putdown_object,
//        object_manipulation_actions::ActionExecutorPutdownObject,
//        continual_planning_executive::ActionExecutorInterface)
PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::PutdownObject,
		continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{

PutdownObject::PutdownObject()
{
	//placement_gen_.reset(new object_surface_placements::PlacementGeneratorSampling(20, 50));
	placement_gen_.reset(
			new object_surface_placements::PlacementGeneratorDiscretization());

	collision_method_ = object_surface_placements::CM_CONTOUR_CONTOUR;
	z_above_table_ = 0.01;

	// better use service calls rather than planningSceneMontior,
	// which needs the initialize the robot description - takes time)
	// psi_.reset(new symbolic_planning_utils::PlanningSceneMonitor());
	psi_.reset(new symbolic_planning_utils::PlanningSceneService());

	ROS_INFO_STREAM(action_name_<<": action executor is ready");
}

PutdownObject::~PutdownObject()
{
}

void PutdownObject::initialize(const std::deque<std::string> & arguments)
{
	ROS_ASSERT(arguments.size() >= 2);
	action_name_ = arguments[0];
	predicate_inspected_recently_ = arguments[1];
}

bool PutdownObject::canExecute(const DurativeAction & a,
		const SymbolicState & currentState) const
{
	return a.name == action_name_;
}

bool PutdownObject::executeBlocking(const DurativeAction & a,
		SymbolicState & currentState)
{
	ROS_ASSERT(a.parameters.size() == 3);
	std::string movable_obj = a.parameters[0];
	std::string arm = a.parameters[1];
	std::string table = a.parameters[2];

	moveit_msgs::CollisionObject surface_object;
	if (!psi_->getObjectFromCollisionObjects(table, surface_object))
	{
		ROS_ERROR_STREAM(action_name_<<": could not find surface_object "<<table);
		return false;
	}

	moveit_msgs::AttachedCollisionObject attached_object;
	if (!psi_->getAttachedObjectFromAttachedCollisionObjects(movable_obj,
			attached_object))
	{
		ROS_ERROR_STREAM(
				action_name_<<": could not find attached_object "<< movable_obj);
		return false;
	}

	std::vector<moveit_msgs::CollisionObject> other_objects;
	forEach(const moveit_msgs::CollisionObject & co, psi_->getCollisionObjects()){
	if(co.id != table)
	other_objects.push_back(co);
}

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
		ROS_ERROR_STREAM(
				action_name_<<": arm group lookup failed. expected right_arm or left_arm; got "<<a.parameters[0]);
		return false;
	}
	arm_group->getCurrentState();

	std::vector<moveit_msgs::PlaceLocation> failed;
	std::vector<moveit_msgs::PlaceLocation> locs =
			placement_gen_->generatePlacements(eef_name, attached_object.object,
					surface_object, other_objects, collision_method_, z_above_table_,
					Eigen::Affine3d::Identity(), true, &failed);

//    ROS_WARN_STREAM("Number of failed place locations: " << failed.size());
//    ROS_WARN_STREAM("eef_name: " << eef_name);
//    ROS_WARN_STREAM("collision_method:" << collision_method_);
//    ROS_WARN_STREAM("z_above_table: " << z_above_table_);

	if (locs.size() == 0)
	{
		ROS_ERROR_STREAM(action_name_<<": could not determine any place location.");
		return false;
	}

// arm_group->setPlannerId("RRTConnectkConfigDefault");
	arm_group->setSupportSurfaceName(table);

	moveit::planning_interface::MoveItErrorCode error_code;
	error_code = arm_group->place(attached_object.object.id, locs);
	ROS_INFO_STREAM(
			"ActionExecutorPutdownObject::" << __func__ <<": Place object " << attached_object.object.id << " action returned " << error_code);

// set predicate *-inspected-recently to false after successful putdown
	currentState.setBooleanPredicate(predicate_inspected_recently_, table, false);
	return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

void PutdownObject::cancelAction()
{
}

}
;

