/*
 * PickupModules.cpp
 *
 *  Created on: Aug 20, 2015
 *      Author: andreas
 */

#include "planner_modules_pr2/pickup_modules.h"
#include "planner_modules_pr2/manipulation_exceptions.h"

VERIFY_INIT_MODULE_DEF(pickup_init);
VERIFY_CONDITIONCHECKER_DEF(can_pickup);

namespace planner_modules_pr2
{
namespace pickup
{
boost::shared_ptr<ModuleParamCache<double> > cost_cache;

double compute_value(
		planning_scene::PlanningScenePtr scene,
		const string& object_name,
		const string& arm_prefix,
		const string& table_name)
{
	try
	{
		ROS_DEBUG_STREAM(__PRETTY_FUNCTION__<<": starting");
		planner_modules_pr2::ManipulationPlanningPtr p = ManipulationPlanning::instance();
		double cost = p->pickup(scene, object_name, arm_prefix, table_name);
		ROS_DEBUG_STREAM(__PRETTY_FUNCTION__<<": done");
		return cost;
	}
	catch (ManipulationException& ex)
	{
		ROS_ERROR("Pickup failed: %s", ex.what());
	}
	catch (std::exception& ex)
	{
		ROS_ERROR("Error: %s", ex.what());
	}
	return modules::INFINITE_COST;
}

string create_cache_key(const string& object,
		const string& arm,
		const string& table,
		const geometry_msgs::Pose2D& robot_pose,
		double torso_position,
		const map<string, geometry_msgs::Pose>& movableObjects,
		const map<string, string>& objectsOnStatic)
{
	ROS_DEBUG_STREAM(__PRETTY_FUNCTION__<<": "<<object<<", "<<arm<<", "<<table);

	// using type for caching, doesn't matter which object it is.
	std::stringstream stream;
	stream << std::fixed << object << arm << table;
	stream << "R" << createPoseParamString(robot_pose, torso_position);
	for (map<string, string>::const_iterator objectIt = objectsOnStatic.begin(); objectIt != objectsOnStatic.end(); objectIt++)
	{
		ROS_DEBUG_STREAM(__PRETTY_FUNCTION__<<": "<<objectIt->first<<" on "<<objectIt->second);
		if (objectIt->second == table)
		{
			const geometry_msgs::Pose& pose = movableObjects.find(objectIt->first)->second;

			string objectIdentifier = objectIt->first;
			string typeName;
			int typeId;
			// assuming that something like cup0 identifies a cup
			// we only put the type of object into the key as it doesn't matter
			// which one is there.
			if(splitNamedId(objectIt->first, typeName, typeId)) {
				objectIdentifier = typeName;
			}

			// the actual values don't matter as long as they are unique for this object
			// cannot use doubles here, as param keys are not allowed to contain '.'
			std::string poseParamString = createPoseParamString(pose);
			stream << objectIdentifier << "AT" << poseParamString;
		}
	}
	return stream.str();
}

}/* pickup */
} /* namespace planner_modules_pr2 */

using namespace planner_modules_pr2;
using namespace planner_modules_pr2::pickup;
void pickup_init(int argc, char** argv)
{
	ROS_INFO_STREAM(__PRETTY_FUNCTION__);
	cost_cache.reset(new ModuleParamCache<double>("pickup/cost"));
}

double can_pickup(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed)
{
	ROS_ASSERT(parameterList.size() == 3);
	const string& object_name = parameterList[0].value;
	const string& arm_name = parameterList[1].value;
	const string arm_prefix = arm_name.substr(0, arm_name.rfind("_arm"));
	const string& table_name = parameterList[2].value;

	TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
	geometry_msgs::Pose robot_pose;
	double torso_position = 0.0;
	psu->readPose(robot_pose, "robot_location", numericalFluentCallback);
	MovableObjectsMap movableObjects;
	GraspedObjectMap graspedObjects;
	ObjectsOnTablesMap objectsOnTables;
	psu->readObjects(predicateCallback, numericalFluentCallback, movableObjects, graspedObjects, objectsOnTables);

	// convert 6D robot pose to 2D
	geometry_msgs::Pose2D robot_pose_2d;
	robot_pose_2d.x = robot_pose.position.x;
	robot_pose_2d.y = robot_pose.position.y;
	robot_pose_2d.theta = tf::getYaw(robot_pose.orientation);

	// cache lookup
	string cache_key = create_cache_key(object_name, arm_name, table_name, robot_pose_2d, torso_position, movableObjects, objectsOnTables);
	double value;
	if (cost_cache->get(cache_key, value))
	{
		return value;
	}

	// compute value
	ros::WallTime compute_start_time = ros::WallTime::now();
	planning_scene::PlanningScenePtr scene = psu->getCurrentScene("robot_location", predicateCallback, numericalFluentCallback);
	value = compute_value(scene, object_name, arm_prefix, table_name);
//	psu->visualize(scene);
	ros::WallTime compute_end_time = ros::WallTime::now();

	// store in cache
	cost_cache->set(cache_key, value, (compute_end_time - compute_start_time).toSec());

	return value;
}

double can_pickup_grounding(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed)
{
	ROS_ASSERT(parameterList.size() == 3);
	const string& object_name = parameterList[0].value;
	const string& arm_name = parameterList[1].value;
	const string arm_prefix = arm_name.substr(0, arm_name.rfind("_arm"));
	const string& table_name = parameterList[2].value;

	TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
	geometry_msgs::Pose2D robot_pose;
	double torso_position = 0.0;
	psu->readRobotPose2D(robot_pose, torso_position, numericalFluentCallback);
	MovableObjectsMap movableObjects;
	GraspedObjectMap graspedObjects;
	ObjectsOnTablesMap objectsOnTables;
	psu->readObjects(predicateCallback, numericalFluentCallback, movableObjects, graspedObjects, objectsOnTables);

	// cache lookup
	string cache_key = create_cache_key(object_name, arm_name, table_name, robot_pose, torso_position, movableObjects, objectsOnTables);
	double value;
	if (cost_cache->get(cache_key, value))
	{
		return value;
	}

	// compute value
	ros::WallTime compute_start_time = ros::WallTime::now();
	planning_scene::PlanningScenePtr scene = psu->getCurrentScene(predicateCallback, numericalFluentCallback);
	psu->visualize(scene);
	value = compute_value(scene, object_name, arm_prefix, table_name);
	ros::WallTime compute_end_time = ros::WallTime::now();

	// store in cache
	cost_cache->set(cache_key, value, (compute_end_time - compute_start_time).toSec());

	return value;
}

int pickup_effect(const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
		vector<double> & writtenVars)
{
	// pickup-object ?o - movable_object ?a - arm ?t - table ?l - manipulation_location
	ROS_ASSERT(parameterList.size() == 3);
	const std::string& movable_object        = parameterList[0].value;
	const std::string& arm                   = parameterList[1].value;
	const std::string& table                 = parameterList[2].value;
//	const std::string& manipulation_location = parameterList[3].value;

	TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
	planning_scene::PlanningScenePtr scene = psu->getCurrentScene("robot_location", predicateCallback, numericalFluentCallback);

	MovableObjectsMap movableObjects;
	GraspedObjectMap graspedObjects;
	ObjectsOnTablesMap objectsOnTables;
	psu->readObjects(predicateCallback, numericalFluentCallback, movableObjects, graspedObjects, objectsOnTables);

	// set object as grasped
	// check if object is in planning scene
	if (movableObjects.find(movable_object) == movableObjects.end() )
	{
		ROS_ERROR("pickup_modules::%s: Could not find movable object: %s in planning scene.", __func__, movable_object.c_str());
		return 1;
	}
	graspedObjects[movable_object] = std::make_pair(arm, movableObjects[movable_object]);

	psu->updateObjects(scene, movableObjects, graspedObjects);
	psu->visualize(scene);
	ROS_WARN("pickup_modules::%s: Visualized Planning scene, wait 10 seconds.", __func__);
	ros::Duration(10.0).sleep();

	moveit_msgs::PlanningScene  ps_msg;
	scene->getPlanningSceneMsg(ps_msg);

	geometry_msgs::Pose object_pose;
	const std::vector<moveit_msgs::AttachedCollisionObject>& attached_objects = ps_msg.robot_state.attached_collision_objects;

	for (std::vector<moveit_msgs::AttachedCollisionObject>::const_iterator it = attached_objects.begin();
			it != attached_objects.end(); it++)
	{
		if (it->object.id == movable_object)
		{
			if (it->object.mesh_poses.size() > 0)
			{
				object_pose = it->object.mesh_poses[0];
				break;
			}
			else if (it->object.primitive_poses.size() > 0)
			{
				object_pose = it->object.primitive_poses[0];
				break;
			}
			else
			{
				ROS_ERROR("Could not find a pose for object %s", movable_object.c_str());
			}
		}
	}

	ROS_ASSERT(writtenVars.size() == 7);
	writtenVars[0] = object_pose.position.x;
	writtenVars[1] = object_pose.position.y;
	writtenVars[2] = object_pose.position.z;
	writtenVars[3] = object_pose.orientation.x;
	writtenVars[4] = object_pose.orientation.y;
	writtenVars[5] = object_pose.orientation.z;
	writtenVars[6] = object_pose.orientation.w;

	return 1;
}


