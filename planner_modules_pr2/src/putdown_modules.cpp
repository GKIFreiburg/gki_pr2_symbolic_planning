/*
 * putdownModules.cpp
 *
 *  Created on: Aug 20, 2015
 *      Author: andreas
 */

#include "planner_modules_pr2/putdown_modules.h"
#include "planner_modules_pr2/manipulation_exceptions.h"

VERIFY_INIT_MODULE_DEF(putdown_init);
VERIFY_CONDITIONCHECKER_DEF(can_putdown);

namespace planner_modules_pr2
{
namespace putdown
{

boost::shared_ptr<ModuleParamCache<double> > cost_cache;
boost::shared_ptr<ModuleParamCache<std::vector<double> > > putdown_poses_cache;

double compute_value(
		planning_scene::PlanningScenePtr scene,
		const string& object_name,
		const string& arm_prefix,
		const string& table)
{
	try
	{
		ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": starting");
		planner_modules_pr2::ManipulationPlanningPtr p = planner_modules_pr2::ManipulationPlanning::instance();
		double cost = p->putdown(scene, object_name, arm_prefix, table);
		ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": done");

		return cost;
	}
	catch (planner_modules_pr2::ManipulationException& ex)
	{
		ROS_ERROR("putdown failed: %s", ex.what());
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

}/* namespace putdown */
} /* namespace planner_modules_pr2 */

using namespace planner_modules_pr2;
using namespace planner_modules_pr2::putdown;
void putdown_init(int argc, char** argv)
{
	ROS_INFO_STREAM(__PRETTY_FUNCTION__);
	cost_cache.reset(new ModuleParamCache<double>("putdown/cost"));
	putdown_poses_cache.reset(new ModuleParamCache<std::vector<double> >("putdown/poses"));
}

double can_putdown(
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

	// needed to create cache key
	TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
	geometry_msgs::Pose robot_pose;
	double torso_position = 0.0;
	psu->readPose(robot_pose, "robot_location", numericalFluentCallback);
	MovableObjectsMap movableObjects;
	GraspedObjectMap graspedObjects;
	ObjectsOnTablesMap objectsOnTables;
	psu->readObjects(predicateCallback, numericalFluentCallback, movableObjects, graspedObjects, objectsOnTables);
	ROS_ASSERT(graspedObjects.size() > 0);

	// convert 6D robot pose to 2D
	geometry_msgs::Pose2D robot_pose_2d;
	robot_pose_2d.x = robot_pose.position.x;
	robot_pose_2d.y = robot_pose.position.y;
	robot_pose_2d.theta = tf::getYaw(robot_pose.orientation);

	// cache
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
	ros::WallTime compute_end_time = ros::WallTime::now();

	psu->visualize(scene);
//	ROS_WARN("1 SHOWING PUTDOWN PLANNING SCENE - WAIT 30 SECONDS");
//	ros::Duration(30.0).sleep();


	// store in cache
	cost_cache->set(cache_key, value, (compute_end_time - compute_start_time).toSec());
	// if value is infinite, meaning putdown failed -> no object in world, therefore return
	if (value == modules::INFINITE_COST)
		return value;

	// store putdown pose of object in cache for effect module
	EigenSTL::vector_Affine3d poses = scene->getWorld()->getObject(object_name)->shape_poses_;
	ROS_ASSERT(poses.size() == 1);
	geometry_msgs::Pose object_pose;
	tf::poseEigenToMsg(poses[0], object_pose);
	std::vector<double> pose;
	pose.resize(7);
	pose[0] = object_pose.position.x;
	pose[1] = object_pose.position.y;
	pose[2] = object_pose.position.z;
	pose[3] = object_pose.orientation.x;
	pose[4] = object_pose.orientation.y;
	pose[5] = object_pose.orientation.z;
	pose[6] = object_pose.orientation.w;

	putdown_poses_cache->set(cache_key, pose, (compute_end_time - compute_start_time).toSec());
	return value;
}

double can_putdown_grounding(
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

	// needed to create cache key
	TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
	geometry_msgs::Pose2D robot_pose;
	double torso_position = 0.0;
	psu->readRobotPose2D(robot_pose, torso_position, numericalFluentCallback);
	MovableObjectsMap movableObjects;
	GraspedObjectMap graspedObjects;
	ObjectsOnTablesMap objectsOnTables;
	psu->readObjects(predicateCallback, numericalFluentCallback, movableObjects, graspedObjects, objectsOnTables);

	// cache
	string cache_key = create_cache_key(object_name, arm_name, table_name, robot_pose, torso_position, movableObjects, objectsOnTables);
	double value;
	if (cost_cache->get(cache_key, value))
	{
		return value;
	}

	// compute value
	ros::WallTime compute_start_time = ros::WallTime::now();
	planning_scene::PlanningScenePtr scene = psu->getCurrentScene(predicateCallback, numericalFluentCallback);
	value = compute_value(scene, object_name, arm_prefix, table_name);
	ros::WallTime compute_end_time = ros::WallTime::now();

	// store in cache
	cost_cache->set(cache_key, value, (compute_end_time - compute_start_time).toSec());
	// if value is infinite, meaning putdown failed -> no object in world, therefore return
	if (value == modules::INFINITE_COST)
		return value;

	// store putdown pose of object in cache for effect module
	EigenSTL::vector_Affine3d poses = scene->getWorld()->getObject(object_name)->shape_poses_;
	ROS_ASSERT(poses.size() == 1);
	geometry_msgs::Pose object_pose;
	tf::poseEigenToMsg(poses[0], object_pose);
	std::vector<double> pose;
	pose.resize(7);
	pose[0] = object_pose.position.x;
	pose[1] = object_pose.position.y;
	pose[2] = object_pose.position.z;
	pose[3] = object_pose.orientation.x;
	pose[4] = object_pose.orientation.y;
	pose[5] = object_pose.orientation.z;
	pose[6] = object_pose.orientation.w;

	putdown_poses_cache->set(cache_key, pose, (compute_end_time - compute_start_time).toSec());

	return value;
}

int putdown_effect(const modules::ParameterList& parameterList,
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

	// needed to create cache key
	TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
	geometry_msgs::Pose robot_pose;
	double torso_position = 0.0;
	psu->readPose(robot_pose, "robot_location", numericalFluentCallback);
	MovableObjectsMap movableObjects;
	GraspedObjectMap graspedObjects;
	ObjectsOnTablesMap objectsOnTables;
	psu->readObjects(predicateCallback, numericalFluentCallback, movableObjects, graspedObjects, objectsOnTables);
	ROS_ASSERT(graspedObjects.size() > 0);

	// convert 6D robot pose to 2D
	geometry_msgs::Pose2D robot_pose_2d;
	robot_pose_2d.x = robot_pose.position.x;
	robot_pose_2d.y = robot_pose.position.y;
	robot_pose_2d.theta = tf::getYaw(robot_pose.orientation);

	// cache
	string cache_key = create_cache_key(object_name, arm_name, table_name, robot_pose_2d, torso_position, movableObjects, objectsOnTables);

	// look in cache
	std::vector<double> object_pose;
	if (!putdown_poses_cache->get(cache_key, object_pose))
	{
		ROS_ERROR("putdown_modules::%s: Could not find previously generated put down pose!", __func__);
		return 0;
	}

	ROS_ASSERT(writtenVars.size() == object_pose.size());
	ROS_ASSERT(writtenVars.size() == 7);
	writtenVars[0] = object_pose[0];
	writtenVars[1] = object_pose[1];
	writtenVars[2] = object_pose[2];
	writtenVars[3] = object_pose[3];
	writtenVars[4] = object_pose[4];
	writtenVars[5] = object_pose[5];
	writtenVars[6] = object_pose[6];
	return 1;
}

int putdown_effect_grounding(const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
		vector<double> & writtenVars)
{
	ROS_ASSERT(parameterList.size() == 3);
	const string& object_name = parameterList[0].value;
	const string& arm_name = parameterList[1].value;
	const string arm_prefix = arm_name.substr(0, arm_name.rfind("_arm"));
	const string& table_name = parameterList[2].value;

	// needed to create cache key
	TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
	geometry_msgs::Pose2D robot_pose;
	double torso_position = 0.0;
	psu->readRobotPose2D(robot_pose, torso_position, numericalFluentCallback);
	MovableObjectsMap movableObjects;
	GraspedObjectMap graspedObjects;
	ObjectsOnTablesMap objectsOnTables;
	psu->readObjects(predicateCallback, numericalFluentCallback, movableObjects, graspedObjects, objectsOnTables);

	// cache
	string cache_key = create_cache_key(object_name, arm_name, table_name, robot_pose, torso_position, movableObjects, objectsOnTables);

	// look in cache
	std::vector<double> object_pose;
	if (!putdown_poses_cache->get(cache_key, object_pose))
	{
		ROS_ERROR("putdown_modules::%s: Could not find previously generated put down pose!", __func__);
		return 0;
	}

	ROS_ASSERT(writtenVars.size() == object_pose.size());
	ROS_ASSERT(writtenVars.size() == 7);
	writtenVars[0] = object_pose[0];
	writtenVars[1] = object_pose[1];
	writtenVars[2] = object_pose[2];
	writtenVars[3] = object_pose[3];
	writtenVars[4] = object_pose[4];
	writtenVars[5] = object_pose[5];
	writtenVars[6] = object_pose[6];

	return 1;
}
