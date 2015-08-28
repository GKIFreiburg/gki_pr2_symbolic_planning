/*
 * ManipulationPlanning.cpp
 *
 *  Created on: Jul 29, 2015
 *      Author: andreas
 */

#include "planner_modules_pr2/manipulation_exceptions.h"
#include "planner_modules_pr2/manipulation_planning.h"
#include <boost/variant.hpp>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

namespace planner_modules_pr2
{

ManipulationPlanningPtr ManipulationPlanning::instance_;

ManipulationPlanningPtr ManipulationPlanning::instance()
{
	if (instance_ == NULL)
	{
		instance_.reset(new ManipulationPlanning());
	}
	return instance_;
}

ManipulationPlanning::ManipulationPlanning()
{
	collision_mode = object_surface_placements::CM_CONTOUR_CONTOUR;
	z_above_table = 0.01;

	ros::NodeHandle nhMoveGroup("move_group");

	scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
	scene_monitor->requestPlanningSceneState();

	grasp_generator.reset(new actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction>("generate_grasps", true));
	ROS_INFO("Waiting for generate_grasps action.");
	grasp_generator->waitForServer();

	placement_genenerator.reset(new object_surface_placements::PlacementGeneratorSampling(20, 50));

	planning_scene::PlanningScenePtr scene;
	{
		planning_scene_monitor::LockedPlanningSceneRO ps(scene_monitor);
		scene = ps->diff();
		scene->decoupleParent();
	}

	planning.reset(new planning_pipeline::PlanningPipeline(scene->getCurrentState().getRobotModel(), nhMoveGroup, "planning_plugin", "request_adapters"));

	pick_place.reset(new pick_place::PickPlace(planning));
	// vis ends up at ~/display_planned_path
	pick_place->displayComputedMotionPlans(true);
	pick_place->displayProcessedGrasps(true);

	state_storage.reset(new moveit_warehouse::RobotStateStorage());
}

ManipulationPlanning::~ManipulationPlanning()
{
}


double ManipulationPlanning::pickup(planning_scene::PlanningScenePtr scene,
			const std::string& object,
			const std::string& arm_prefix,
			const std::string& support_surface)
{
	moveit_msgs::PickupGoal goal;
	goal.target_name = object;
	goal.group_name = arm_prefix + "_arm";
	goal.allowed_planning_time = 5.0;
	goal.support_surface_name = support_surface;
	goal.allow_gripper_support_collision = false;
	goal.end_effector = arm_prefix + "_eef";

	fillGrasps(scene, goal.target_name, arm_prefix, goal.possible_grasps);

	pick_place::PickPlanPtr plan;
	plan = pick_place->planPick(scene, goal);
	const std::vector<pick_place::ManipulationPlanPtr>& success = plan->getSuccessfulManipulationPlans();
	if (success.empty())
	{
		throw PickupPlanFailedException(plan->getErrorCode());
	}
	pick_place::ManipulationPlanPtr manipulation_plan = success.back();

	return applyManipulationPlan(scene, manipulation_plan, manipulation_plan->approach_posture_);
}

double ManipulationPlanning::putdown(planning_scene::PlanningScenePtr scene,
			const std::string& object,
			const std::string& arm_prefix,
			const std::string& support_surface)
{
	moveit_msgs::PlaceGoal goal;
	goal.attached_object_name = object;
	goal.group_name = arm_prefix + "_arm";
	goal.allowed_planning_time = 5.0;
	goal.support_surface_name = support_surface;
	goal.allow_gripper_support_collision = false;
	goal.place_eef = false;

	fillPlacements(scene, object, arm_prefix, support_surface, goal.place_locations);

	pick_place::PlacePlanPtr plan;
	plan = pick_place->planPlace(scene, goal);
	const std::vector<pick_place::ManipulationPlanPtr>& success = plan->getSuccessfulManipulationPlans();
	if (success.empty())
	{
		throw PickupPlanFailedException(plan->getErrorCode());
	}
	pick_place::ManipulationPlanPtr manipulation_plan = success.back();
	return applyManipulationPlan(scene, manipulation_plan, manipulation_plan->retreat_posture_);
}

double ManipulationPlanning::applyManipulationPlan(planning_scene::PlanningScenePtr scene,
		const pick_place::ManipulationPlanPtr manipulation_plan,
		const trajectory_msgs::JointTrajectory detach_posture)
{
	double cost = 0;
	forEach(const plan_execution::ExecutableTrajectory& traj, manipulation_plan->trajectories_)
	{
		cost += traj.trajectory_->getWaypointDurationFromStart(traj.trajectory_->getWayPointCount());
		ROS_DEBUG("At Stage: %s", traj.description_.c_str());
		ROS_DEBUG("Has effect_on_success_: %d ?", traj.effect_on_success_?1:0);
		ROS_DEBUG("Setting planning scene to resulting state");
		scene->setCurrentState(traj.trajectory_->getLastWayPoint());
		if(traj.effect_on_success_)
		{
			// the "grasp" effect_on_success_ is to processAttachedCollisionObjectMsg with the diff_attached_object_.
			// effect_on_success_ implements via the monitor, which we can't do as we have another
			// planning scene than the tracked one.
			if(traj.description_ == "grasp")
			{
				moveit_msgs::AttachedCollisionObject obj = manipulation_plan->shared_data_->diff_attached_object_;
				obj.detach_posture = detach_posture;
				ROS_DEBUG_STREAM("PROCESSING" << obj);
				scene->processAttachedCollisionObjectMsg(obj);
			}
		}
	}
	return cost;
}

void ManipulationPlanning::fillGrasps(
		const planning_scene::PlanningScenePtr scene,
		const std::string& object,
		const std::string& arm,
		std::vector<moveit_msgs::Grasp>& grasps_to_fill)
{
	// 1. get the object to query grasps for
	moveit_msgs::CollisionObject co = getCollisionObjectFromPlanningScene(scene, object);

	// 2. get grasps
	grasp_provider_msgs::GenerateGraspsGoal grasps;
	grasps.collision_object = co;
	grasps.eef_group_name = arm + "_gripper";

	grasp_generator->sendGoal(grasps);
	ROS_INFO("Waiting for grasps");
	if (!grasp_generator->waitForResult(ros::Duration(3.0)))
	{
		throw GeneratingGraspsTimeoutException();
	}
	else if (grasp_generator->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		throw GeneratingGraspsFailedException();
	}
	if (grasp_generator->getResult()->grasps.empty())
	{
		throw ZeroGraspsGeneratedException();
	}
	grasps_to_fill = grasp_generator->getResult()->grasps;
	ROS_INFO("fillGrasps: %zu grasps", grasp_generator->getResult()->grasps.size());
}

void ManipulationPlanning::fillPlacements(
		const planning_scene::PlanningScenePtr scene,
		const std::string& object,
		const std::string& arm_prefix,
		const std::string& support_surface,
		std::vector<moveit_msgs::PlaceLocation>& place_locations)
{
	moveit_msgs::PlanningScene psMsg;
	scene->getPlanningSceneMsg(psMsg);

	// get attached_object object from arm_prefix as Msg
	moveit_msgs::CollisionObject attached_object;
	forEach(const moveit_msgs::AttachedCollisionObject & aco, psMsg.robot_state.attached_collision_objects)
	{
		if(aco.object.id == object)
		{
			attached_object = aco.object;
			break;
		}
	}
	//attached_object = getCollisionObjectFromPlanningScene(scene, object);
	moveit_msgs::CollisionObject surface_object = getCollisionObjectFromPlanningScene(scene, support_surface);

	std::vector<moveit_msgs::CollisionObject> other_objects;
	// get surface object from g_table as Msg
	// get other_objects from scene - g_table as Msgs
	forEach(const moveit_msgs::CollisionObject & co, psMsg.world.collision_objects)
	{
		if(co.id == support_surface)
		{
			surface_object = co;
		}
		else
		{
			other_objects.push_back(co);
		}
	}

	place_locations = placement_genenerator->generatePlacements(arm_prefix + "_gripper", attached_object, surface_object, other_objects, collision_mode, z_above_table, Eigen::Affine3d::Identity(), true, NULL);
	if (place_locations.empty())
	{
		throw ZeroPlacementsGeneratedException();
	}
}

moveit_msgs::CollisionObject ManipulationPlanning::getCollisionObjectFromPlanningScene(
		const planning_scene::PlanningScenePtr scene,
		const std::string& id)
{
	const EigenSTL::vector_Affine3d* shape_transforms = NULL;
	const std::vector<shapes::ShapeConstPtr>* shapes = NULL;

	collision_detection::CollisionWorld::ObjectConstPtr obj = scene->getWorld()->getObject(id);
	if (obj)
	{
		shapes = &obj->shapes_;
		shape_transforms = &obj->shape_poses_;
	}
	else
	{
		// check if it's attached
		const robot_state::RobotState& state = scene->getCurrentState();
		std::vector<const moveit::core::AttachedBody*> attached_bodies;
		state.getAttachedBodies(attached_bodies);
		forEach(const moveit::core::AttachedBody* ab, attached_bodies)
		{
			if(ab->getName() != id)
			{
				continue;
			}
			// found it
			shapes = &ab->getShapes();
			shape_transforms = &ab->getGlobalCollisionBodyTransforms();
			break;
		}
	}

	if (shapes == NULL || shape_transforms == NULL)
	{
		throw ObjectNotFoundInSceneException(id);
	}

	return createCollisionObject(id, scene, *shapes, *shape_transforms);
}

// next two are basically a copy from planning_scene.cpp
// because that's private in there
class ShapeVisitorAddToCollisionObject: public boost::static_visitor<void>
{
public:
	ShapeVisitorAddToCollisionObject(moveit_msgs::CollisionObject* obj) :
			boost::static_visitor<void>(), obj_(obj), pose_(NULL)
	{
	}

	void setPoseMessage(const geometry_msgs::Pose* pose)
	{
		pose_ = pose;
	}

	void operator()(const shape_msgs::Plane &shape_msg) const
	{
		obj_->planes.push_back(shape_msg);
		obj_->plane_poses.push_back(*pose_);
	}

	void operator()(const shape_msgs::Mesh &shape_msg) const
	{
		obj_->meshes.push_back(shape_msg);
		obj_->mesh_poses.push_back(*pose_);
	}

	void operator()(const shape_msgs::SolidPrimitive &shape_msg) const
	{
		obj_->primitives.push_back(shape_msg);
		obj_->primitive_poses.push_back(*pose_);
	}

private:

	moveit_msgs::CollisionObject* obj_;
	const geometry_msgs::Pose* pose_;
};

moveit_msgs::CollisionObject ManipulationPlanning::createCollisionObject(
		const std::string& name,
		const planning_scene::PlanningScenePtr scene,
		const std::vector<shapes::ShapeConstPtr>& shapes,
		const EigenSTL::vector_Affine3d& shape_transforms)
{
	moveit_msgs::CollisionObject co;
	co.id = name;
	co.header.frame_id = scene->getPlanningFrame();
	co.operation = moveit_msgs::CollisionObject::ADD;

	ShapeVisitorAddToCollisionObject sv(&co);
	for (std::size_t j = 0; j < shapes.size(); ++j)
	{
		shapes::ShapeMsg sm;
		if (constructMsgFromShape(shapes[j].get(), sm))
		{
			geometry_msgs::Pose p;
			tf::poseEigenToMsg(shape_transforms[j], p);

			sv.setPoseMessage(&p);
			boost::apply_visitor(sv, sm);
		}
	}

	if (!co.primitives.empty() || !co.meshes.empty() || !co.planes.empty())
	{
		if (scene->hasObjectType(co.id))
		{
			co.type = scene->getObjectType(co.id);
		}
	}
	return co;
}

}




