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
VERIFY_CONDITIONCHECKER_DEF(pickup_cost);

ros::NodeHandlePtr node;
ModuleParamCache<double> costCache;

void pickup_init(int argc, char** argv)
{
	ROS_INFO_STREAM(__PRETTY_FUNCTION__);
	node.reset(new ros::NodeHandle());
	costCache.initialize("pickup/cost", node.get());
}

double pickup(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed)
{
	ROS_ASSERT(parameterList.size() >= 3);
	const string& object_name = parameterList[0].value;
	const string& arm_name = parameterList[1].value;
	const string arm_prefix = arm_name.substr(0, arm_name.rfind("_arm"));
	const string& table_name = parameterList[2].value;

	TidyupPlanningSceneUpdater* updater = TidyupPlanningSceneUpdater::instance();
	geometry_msgs::Pose2D robot_pose;
	double torsoPosition = 0.0;
	updater->readRobotPose2D(robot_pose, torsoPosition, numericalFluentCallback);
	ROS_INFO_STREAM(__func__<<": torso: "<< torsoPosition);
	MovableObjectsMap movableObjects;
	GraspedObjectMap graspedObjects;
	ObjectsOnTablesMap objectsOnTables;
	updater->readObjects(predicateCallback, numericalFluentCallback, movableObjects, graspedObjects, objectsOnTables);

	// cache
	string key = compute_pickup_cache_key(object_name, arm_name, table_name, robot_pose, movableObjects, objectsOnTables);
	double cost;
	if (costCache.get(key, cost))
	{
		ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": cache hit, cost: "<<cost);
		return cost;
	}

	planning_scene::PlanningScenePtr scene = updater->getEmptyScene();
	updater->updateRobotPose2D(scene, robot_pose, torsoPosition);
	updater->updateObjects(scene, movableObjects, graspedObjects);

	try
	{
		ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": starting");
		planner_modules_pr2::ManipulationPlanningPtr p = planner_modules_pr2::ManipulationPlanning::instance();
		double cost = p->pickup(scene, object_name, arm_prefix, table_name);
		ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": done");
		return cost;
	}
	catch (planner_modules_pr2::ManipulationException& ex)
	{
		ROS_ERROR("Pickup failed: %s", ex.what());
	}
	catch (std::exception& ex)
	{
		ROS_ERROR("Error: %s", ex.what());
	}
	return modules::INFINITE_COST;
}

string compute_pickup_cache_key(const string& object,
		const string& arm,
		const string& table,
		const geometry_msgs::Pose2D& robot_pose,
		const map<string, geometry_msgs::Pose>& movableObjects,
		const map<string, string>& objectsOnStatic)
{
	ROS_DEBUG_STREAM(__PRETTY_FUNCTION__<<": "<<object<<", "<<arm<<", "<<table);

	// using type for caching, doesn't matter which object it is.
	std::stringstream stream;
	stream << std::fixed << object << arm << table;
	stream << "R" << createPoseParamString(robot_pose);
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

double pickup_cost(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed)
{
	ROS_INFO_STREAM(__PRETTY_FUNCTION__);
	return pickup(parameterList, predicateCallback, numericalFluentCallback, relaxed);
}

double can_pickup(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed)
{
	ROS_INFO_STREAM(__PRETTY_FUNCTION__);
	return pickup(parameterList, predicateCallback, numericalFluentCallback, relaxed);
}

