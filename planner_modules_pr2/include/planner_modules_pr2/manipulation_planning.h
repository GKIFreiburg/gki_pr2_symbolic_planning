/*
 * ManipulationPlanning.h
 *
 *  Created on: Jul 29, 2015
 *      Author: andreas
 */

#ifndef MANIPULATIONPLANNING_H_
#define MANIPULATIONPLANNING_H_

#include <moveit/warehouse/state_storage.h>

#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/pick_place/pick_place.h>
#include <moveit/move_group/move_group_capability.h>

#include <grasp_provider_msgs/GenerateGraspsAction.h>

#include <object_surface_placements/placements_visualization.h>
#include <object_surface_placements/placement_generator_discretization.h>
#include <object_surface_placements/placement_generator_sampling.h>

namespace planner_modules_pr2
{

class ManipulationPlanning;
typedef boost::shared_ptr<ManipulationPlanning> ManipulationPlanningPtr;
class ManipulationPlanning
{
public:
	static ManipulationPlanningPtr instance();
	virtual ~ManipulationPlanning();

	double pickup(planning_scene::PlanningScenePtr scene,
			const std::string& object,
			const std::string& arm_prefix,
			const std::string& support_surface);

	double putdown(planning_scene::PlanningScenePtr scene,
			const std::string& object,
			const std::string& arm_prefix,
			const std::string& support_surface);

private:
	static ManipulationPlanningPtr instance_;
	ManipulationPlanning();

	double applyManipulationPlan(planning_scene::PlanningScenePtr scene,
			const pick_place::ManipulationPlanPtr manipulation_plan,
			const trajectory_msgs::JointTrajectory detach_posture);

	void fillGrasps(const planning_scene::PlanningScenePtr scene,
			const std::string& object,
			const std::string& arm_prefix,
			std::vector<moveit_msgs::Grasp>& grasps_to_fill);

	void fillPlacements(
			const planning_scene::PlanningScenePtr scene,
			const std::string& object,
			const std::string& arm_prefix,
			const std::string& support_surface,
			std::vector<moveit_msgs::PlaceLocation>& place_locations);

	moveit_msgs::CollisionObject getCollisionObjectFromPlanningScene(
			const planning_scene::PlanningScenePtr scene,
			const std::string& id);
	moveit_msgs::CollisionObject createCollisionObject(
			const std::string& name,
			const planning_scene::PlanningScenePtr scene,
			const std::vector<shapes::ShapeConstPtr>& shapes,
			const EigenSTL::vector_Affine3d& shape_transforms);

	boost::shared_ptr<object_surface_placements::PlacementGenerator> placement_genenerator;
	boost::shared_ptr<actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction> > grasp_generator;

	boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> scene_monitor;
	planning_pipeline::PlanningPipelinePtr planning;
	boost::shared_ptr<moveit_warehouse::RobotStateStorage> state_storage;
	pick_place::PickPlacePtr pick_place;

	object_surface_placements::CollisionMethod collision_mode;
	double z_above_table;

	std::map<std::string, moveit_msgs::CollisionObject> object_types_;

};


}
#endif /* MANIPULATIONPLANNING_H_ */

