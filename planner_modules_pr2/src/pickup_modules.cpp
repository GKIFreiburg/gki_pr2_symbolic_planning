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
boost::shared_ptr<ModuleParamCache<std::vector<double> > > pickup_grasps_cache;

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
	pickup_grasps_cache.reset(new ModuleParamCache<std::vector<double> >("pickup/grasps"));
}

double can_pickup(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed)
{
	ROS_ASSERT(parameterList.size() == 4);
	const string& object_name = parameterList[0].value;
	const string& arm_name = parameterList[1].value;
	const string arm_prefix = arm_name.substr(0, arm_name.rfind("_arm"));
	const string& table_name = parameterList[2].value;
	const string& manipulation_location = parameterList[3].value;

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
	psu->visualize(scene);
	ros::WallTime compute_end_time = ros::WallTime::now();

	// store in cache
	cost_cache->set(cache_key, value, (compute_end_time - compute_start_time).toSec());
	// if value is infinite, meaning pick up failed -> no attached object, therefore return
	if (value == modules::INFINITE_COST)
		return value;

	// store pickup grasp of object in cache for effect module
	EigenSTL::vector_Affine3d attach_poses = scene->getCurrentState().getAttachedBody(object_name)->getFixedTransforms();
	scene->getCurrentState().getAttachedBody(object_name)->getAttachedLinkName();
	ROS_ASSERT(attach_poses.size() == 1);
	geometry_msgs::Pose attach_pose;
	tf::poseEigenToMsg(attach_poses[0], attach_pose);
//	ROS_WARN_STREAM("ATTACH POSE" << attach_pose);
	std::vector<double> pose;
	pose.resize(7);
	pose[0] = attach_pose.position.x;
	pose[1] = attach_pose.position.y;
	pose[2] = attach_pose.position.z;
	pose[3] = attach_pose.orientation.x;
	pose[4] = attach_pose.orientation.y;
	pose[5] = attach_pose.orientation.z;
	pose[6] = attach_pose.orientation.w;

	pickup_grasps_cache->set(cache_key, pose, (compute_end_time - compute_start_time).toSec());
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
	ROS_ASSERT(parameterList.size() == 4);
	const string& object_name = parameterList[0].value;
	const string& arm_name = parameterList[1].value;
	const string arm_prefix = arm_name.substr(0, arm_name.rfind("_arm"));
	const string& table_name = parameterList[2].value;
	const string& manipulation_location = parameterList[3].value;

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

	// loop in cache
	std::vector<double> attach_pose;
	if (!pickup_grasps_cache->get(cache_key, attach_pose))
	{
		ROS_ERROR("pickup_modules::%s: Could not find previously generated pick up grasp!", __func__);
		return 0;
	}

	ROS_ASSERT(writtenVars.size() == attach_pose.size());
	ROS_ASSERT(writtenVars.size() == 7);
	writtenVars[0] = attach_pose[0];
	writtenVars[1] = attach_pose[1];
	writtenVars[2] = attach_pose[2];
	writtenVars[3] = attach_pose[3];
	writtenVars[4] = attach_pose[4];
	writtenVars[5] = attach_pose[5];
	writtenVars[6] = attach_pose[6];
	return 1;
}

//
//int pickup_effect_grounding(const modules::ParameterList& parameterList,
//		modules::predicateCallbackType predicateCallback,
//		modules::numericalFluentCallbackType numericalFluentCallback,
//		int relaxed,
//		vector<double> & writtenVars)
//{
//
//}


