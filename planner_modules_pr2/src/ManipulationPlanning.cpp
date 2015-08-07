/*
 * ManipulationPlanning.cpp
 *
 *  Created on: Jul 29, 2015
 *      Author: andreas
 */

#include "planner_modules_pr2/ManipulationExceptions.h"
#include "planner_modules_pr2/ManipulationPlanning.h"
#include <boost/variant.hpp>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

namespace planner_modules_pr2
{

boost::shared_ptr<ManipulationPlanning> ManipulationPlanning::instance_;

boost::weak_ptr<ManipulationPlanning> ManipulationPlanning::instance()
{
	if (instance_ == NULL)
	{
		instance_.reset(new ManipulationPlanning());
	}
	return instance_;
}

ManipulationPlanning::ManipulationPlanning()
{
	g_CollisionMode = object_surface_placements::CM_CONTOUR_CONTOUR;
	z_above_table = 0.01;

	ros::NodeHandle nh;
	ros::NodeHandle nhMoveGroup("move_group");

	pubPlanningScene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene_reasoning", 1);
	pubDisplayTraj = nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path_reasoning", 1, true);

	g_psm.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
	g_psm->requestPlanningSceneState();

	g_generate_grasps.reset(new actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction>("generate_grasps", true));
	ROS_INFO("Waiting for generate_grasps action.");
	g_generate_grasps->waitForServer();

	g_placement_gen.reset(new object_surface_placements::PlacementGeneratorSampling(20, 50));

	planning_scene::PlanningScenePtr scene;
	{
		planning_scene_monitor::LockedPlanningSceneRO ps(g_psm);
		scene = ps->diff();
		scene->decoupleParent();
	}

	g_planning.reset(new planning_pipeline::PlanningPipeline(scene->getCurrentState().getRobotModel(), nhMoveGroup, "planning_plugin", "request_adapters"));

	g_pick_place.reset(new pick_place::PickPlace(g_planning));
	// vis ends up at ~/display_planned_path
	g_pick_place->displayComputedMotionPlans(true);
	g_pick_place->displayProcessedGrasps(true);

	g_state_storage.reset(new moveit_warehouse::RobotStateStorage());
}

ManipulationPlanning::~ManipulationPlanning()
{
}

void ManipulationPlanning::pickup(planning_scene::PlanningScenePtr psc,
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

	fillGrasps(psc, goal.target_name, arm_prefix, goal.possible_grasps);
	planPickupAndUpdateScene(psc, goal);
}

void ManipulationPlanning::planPickupAndUpdateScene(planning_scene::PlanningScenePtr scene, const moveit_msgs::PickupGoal& goal)
{
    pick_place::PickPlanPtr plan;
    plan = g_pick_place->planPick(scene, goal);
	const std::vector<pick_place::ManipulationPlanPtr> &success = plan->getSuccessfulManipulationPlans();
	if (success.empty())
	{
		throw PickupPlanFailedException(plan->getErrorCode());
	}
	pick_place::ManipulationPlanPtr manipulation_plan = success.back();
	applyManipulationPlan(scene, manipulation_plan, manipulation_plan->approach_posture_);
}

void ManipulationPlanning::applyManipulationPlan(planning_scene::PlanningScenePtr scene,
		const pick_place::ManipulationPlanPtr manipulation_plan,
		const trajectory_msgs::JointTrajectory detach_posture)
{
	forEach(const plan_execution::ExecutableTrajectory & traj, manipulation_plan->trajectories_)
	{
		ROS_INFO("At Stage: %s", traj.description_.c_str());
		//visualizeTrajectory(*traj.trajectory_);
		ROS_INFO("Has effect_on_success_: %d ?", traj.effect_on_success_?1:0);
		//ros::Duration(3.0).sleep();
		ROS_INFO("Setting planning scene to resulting state");
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
				ROS_INFO_STREAM("PROCESSING" << obj);
				scene->processAttachedCollisionObjectMsg(obj);
			}
		}
	}
}

void ManipulationPlanning::fillGrasps(const planning_scene::PlanningScenePtr & scene,
		const std::string & object,
		const std::string & arm,
		std::vector<moveit_msgs::Grasp> & grasps_to_fill)
{
	// 1. get the object to query grasps for
	moveit_msgs::CollisionObject co = getCollisionObjectFromPlanningScene(scene, object);

	// 2. get grasps
	grasp_provider_msgs::GenerateGraspsGoal grasps;
	grasps.collision_object = co;
	grasps.eef_group_name = arm + "_gripper";

	g_generate_grasps->sendGoal(grasps);
	ROS_INFO("Waiting for grasps");
	if (!g_generate_grasps->waitForResult(ros::Duration(3.0)))
	{
		throw GeneratingGraspsTimeoutException();
	}
	else if (g_generate_grasps->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		throw GeneratingGraspsFailedException();
	}
	if (g_generate_grasps->getResult()->grasps.empty())
	{
		throw ZeroGraspsGeneratedException();
	}
	grasps_to_fill = g_generate_grasps->getResult()->grasps;
	ROS_INFO("fillGrasps: %zu grasps", g_generate_grasps->getResult()->grasps.size());
}

moveit_msgs::CollisionObject ManipulationPlanning::getCollisionObjectFromPlanningScene(const planning_scene::PlanningScenePtr& scene, const std::string& id)
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
		const robot_state::RobotState & state = scene->getCurrentState();
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

		if (shapes == NULL || shape_transforms == NULL)
		{
			throw ObjectNotFoundInSceneException(id);
		}
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

moveit_msgs::CollisionObject ManipulationPlanning::createCollisionObject(const std::string& name,
	const planning_scene::PlanningScenePtr& scene,
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




