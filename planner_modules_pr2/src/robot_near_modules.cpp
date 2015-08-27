/*
 * robot_near_modules.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: andreas
 */

#include "planner_modules_pr2/robot_near_modules.h"
#include <moveit/collision_detection/world.h>
#include <tf_conversions/tf_eigen.h>
#include "planner_modules_pr2/inverse_reachability_maps.h"

VERIFY_INIT_MODULE_DEF(robot_near_table_init);
VERIFY_CONDITIONCHECKER_DEF(robot_near_table);

namespace planner_modules_pr2
{
	boost::shared_ptr<ModuleParamCache<bool> > robotNearTableConditionCache;

	double compute_robot_near_table(planning_scene::PlanningScenePtr scene, const string& table)
	{
		double result = INFINITE_COST;
		// fetch surface pose from symbolic state
		collision_detection::World::ObjectConstPtr table_obj = scene->getWorld()->getObject(table);

		const robot_state::RobotState& robotState = scene->getCurrentState();
		// get torso link pose in map frame
		Eigen::Affine3d transform_map_torso_eigen = robotState.getGlobalLinkTransform("torso_lift_joint");
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
			ROS_ERROR("drive_pose_module::%s: Could not find inverse reachability map for table: %s", __func__, table.c_str());
			return modules::INFINITE_COST;
		}
		TableInverseReachabilityConstPtr irm = it->second;
		// look if there is a match in inv cap map for given torso position in table frame
		InverseCapability inv = irm->inverse_reachability->getNodeInverseCapability(transform_table_torso.getOrigin().x(),
				transform_table_torso.getOrigin().y(),
				transform_table_torso.getOrigin().z());

		const std::map<double, double>& thetas = inv.getThetasPercent();

		// if there exists a theta, means we have an inverse reachability index,
		// indicating that a part of the table can be reached -> we are close to table and return 0.0 (= true)
		if (thetas.size() > 0)
		{
			ROS_INFO_STREAM(__func__<<": Robot is near "<<table);
			result = 0.0;
		}
		else
		{
			ROS_INFO_STREAM(__func__<<": Robot is not near "<<table);
			result = modules::INFINITE_COST;
		}

		return result;
	}

} /* namespace planner_modules_pr2 */

void robot_near_table_init(int argc, char** argv)
{
	ROS_INFO_STREAM(__PRETTY_FUNCTION__);
	planner_modules_pr2::robotNearTableConditionCache.reset(new ModuleParamCache<double>("robot_near_table/condition"));
}

double robot_near_table(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	ROS_ASSERT(parameterList.size() == 1);
	std::string table = parameterList[0].value;
	planner_modules_pr2::TidyupPlanningSceneUpdaterPtr tpsu = planner_modules_pr2::TidyupPlanningSceneUpdater::instance();
	planning_scene::PlanningScenePtr scene = planner_modules_pr2::TidyupPlanningSceneUpdater::instance()->getCurrentScene(predicateCallback, numericalFluentCallback);
	return planner_modules_pr2::compute_robot_near_table(scene, table);
}
