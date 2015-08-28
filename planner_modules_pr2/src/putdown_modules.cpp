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
}

double can_putdown(
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

	return value;
}

