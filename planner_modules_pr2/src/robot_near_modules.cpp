/*
 * robot_near_modules.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: andreas
 */

#include "planner_modules_pr2/robot_near_modules.h"
#include <moveit/collision_detection/world.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose2D.h>
#include "planner_modules_pr2/inverse_reachability_maps.h"
#include "planner_modules_pr2/module_param_cache.h"

VERIFY_INIT_MODULE_DEF(robot_near_table_init);
VERIFY_CONDITIONCHECKER_DEF(robot_near_table);

namespace planner_modules_pr2
{
namespace robot_near_table
{
boost::shared_ptr< ModuleParamCache<double> > cost_cache;

double compute_value(planning_scene::PlanningScenePtr scene, const string& table)
{
	double result = INFINITE_COST;
	// fetch surface pose from symbolic state
	collision_detection::World::ObjectConstPtr table_obj = scene->getWorld()->getObject(table);

	const robot_state::RobotState& robotState = scene->getCurrentState();
	// get torso link pose in map frame
	Eigen::Affine3d transform_map_torso_eigen = robotState.getGlobalLinkTransform("torso_lift_link");
	tf::Transform transform_map_torso;
	tf::transformEigenToTF(transform_map_torso_eigen, transform_map_torso);

	// fetch torso pose and convert into table frame
	tf::Pose transform_map_table;
	tf::Pose transform_table_torso;
	tf::poseEigenToTF(table_obj->shape_poses_.front(), transform_map_table);

	// transform from table to torso (torso pose in table frame)
	transform_table_torso = transform_map_table.inverseTimes(transform_map_torso);

	// fetch matching inverse surface reachability map
	InverseReachabilityMap::const_iterator it = InverseReachabilityMaps::instance()->find(table);
	if (it == InverseReachabilityMaps::instance()->end())
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__<<": Could not find inverse reachability map for "<<table);
		return modules::INFINITE_COST;
	}
	TableInverseReachabilityConstPtr irm = it->second;
	// look if there is a match in inv cap map for given torso position in table frame
	InverseCapability inv = irm->inverse_reachability->getNodeInverseCapability(transform_table_torso.getOrigin().x(), transform_table_torso.getOrigin().y(), transform_table_torso.getOrigin().z());

	const std::map<double, double>& thetas = inv.getThetasPercent();

	// if there exists a theta, means we have an inverse reachability index,
	// indicating that a part of the table can be reached -> we are close to table and return 0.0 (= true)
	if (thetas.size() > 0)
	{
		ROS_DEBUG_STREAM(__PRETTY_FUNCTION__<<": Robot is near "<<table);
		result = 0.0;
	}
	else
	{
		ROS_DEBUG_STREAM(__PRETTY_FUNCTION__<<": Robot is not near "<<table);
		result = modules::INFINITE_COST;
	}

	return result;
}

string create_cache_key(const string& table, const geometry_msgs::Pose2D& robot_pose, double torso_position)
{
	std::string robot_pose_string = createPoseParamString(robot_pose, torso_position);
	return table+robot_pose_string;
}

} /* namespace robot_near_table */

} /* namespace planner_modules_pr2 */

using namespace planner_modules_pr2;
using namespace planner_modules_pr2::robot_near_table;
void robot_near_table_init(int argc, char** argv)
{
	ROS_INFO_STREAM(__PRETTY_FUNCTION__);
	cost_cache.reset(new ModuleParamCache<double>("robot_near_table/condition"));
}

double robot_near_table(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	ROS_ASSERT(parameterList.size() == 1);
	std::string table = parameterList[0].value;
	planner_modules_pr2::TidyupPlanningSceneUpdaterPtr tpsu = TidyupPlanningSceneUpdater::instance();

	// cache lookup
	geometry_msgs::Pose2D robot_pose;
	double torso_position = 0;
	tpsu->readRobotPose2D(robot_pose, torso_position, numericalFluentCallback);
	std::string cache_key = create_cache_key(table, robot_pose, torso_position);
	double value;
	if (cost_cache->get(cache_key, value))
	{
		return value;
	}

	// compute value
	ros::WallTime compute_start_time = ros::WallTime::now();
	planning_scene::PlanningScenePtr scene = tpsu->getCurrentScene(predicateCallback, numericalFluentCallback);
	value = compute_value(scene, table);
	ros::WallTime compute_end_time = ros::WallTime::now();

	// store in cache
	cost_cache->set(cache_key, value, (compute_end_time - compute_start_time).toSec());

	return value;
}
