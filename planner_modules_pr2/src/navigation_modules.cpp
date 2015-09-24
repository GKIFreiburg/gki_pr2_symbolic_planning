
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <utility>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

#include <gki_3dnav_planner/3dnav_planner.h>

#include "planner_modules_pr2/module_param_cache.h"
#include "planner_modules_pr2/drive_pose_module.h"
#include "planner_modules_pr2/navigation_modules.h"
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"

using namespace modules;


VERIFY_CONDITIONCHECKER_DEF(navigation_cost);
VERIFY_APPLYEFFECT_DEF(navigation_effect);

namespace planner_modules_pr2
{
namespace navigation
{

// Distance measured from ground when torso is at minimum (= not lifted)
const double MIN_TORSO_POSITION = 0.802;

double cost_factor = 10;

/// Plan requests are issued using this frame - so the poses from the planner are given in this frame (e.g. map)
std::string world_frame;

double linear_velocity = 0.3;
double angular_velocity = angles::from_degrees(30);

// Using a cache of queried path costs to prevent calling the path planning service multiple times
boost::shared_ptr<ModuleParamCache<double> > cost_cache;

boost::shared_ptr<tf::TransformListener> tf_listener;
boost::shared_ptr<costmap_2d::Costmap2DROS> costmap;
boost::shared_ptr<gki_3dnav_planner::GKI3dNavPlanner> path_planner;

string create_cache_key(
		const geometry_msgs::Pose & startPose,
		const geometry_msgs::Pose & goalPose)
{
	std::string startPoseStr = createPoseParamString(startPose, 0.01, 0.02);
	std::string goalPoseStr = createPoseParamString(goalPose, 0.01, 0.02);

	if (startPoseStr < goalPoseStr)
	{
		return startPoseStr + "__" + goalPoseStr;
	}
	else
	{
		return goalPoseStr + "__" + startPoseStr;
	}
}

double get_plan_cost(const std::vector<geometry_msgs::PoseStamped>& plan)
{
	if(plan.empty())
		return 0;

	double time = 0;
	geometry_msgs::PoseStamped lastPose = plan[0];
	forEach(const geometry_msgs::PoseStamped& p, plan)
	{
		double linear_distance = hypot(lastPose.pose.position.x - p.pose.position.x, lastPose.pose.position.y - p.pose.position.y);
		double angular_distance = fabs(angles::normalize_angle(tf::getYaw(p.pose.orientation) - tf::getYaw(lastPose.pose.orientation)));
		double t = fmax(linear_distance/linear_velocity, angular_distance/angular_velocity);
		time += t;

		lastPose = p;
	}
	return cost_factor * time;
}

double compute_value(planning_scene::PlanningScenePtr scene, nav_msgs::GetPlan& srv)
{
	path_planner->makePlan(scene, srv.request.goal, srv.response.plan.poses);
	if (!srv.response.plan.poses.empty())
	{
		// get plan cost
		return get_plan_cost(srv.response.plan.poses);
	}
	return INFINITE_COST;
}

} /* namespace planner_modules_pr2 */

} /* namespace navigation */

using namespace planner_modules_pr2;
using namespace planner_modules_pr2::navigation;

void navigation_init(int argc, char** argv)
{
	tf_listener.reset(new tf::TransformListener());
	costmap.reset(new costmap_2d::Costmap2DROS("global_costmap", *(tf_listener.get())));
	world_frame = costmap->getGlobalFrameID();
	path_planner.reset(new gki_3dnav_planner::GKI3dNavPlanner("GKI3dNavPlanner", costmap.get()));

	ros::NodeHandle nhPriv("~");
	nhPriv.param("trans_speed", linear_velocity, linear_velocity);
	nhPriv.param("rot_speed", angular_velocity, angular_velocity);

	cost_cache.reset(new ModuleParamCache<double>("navigation/cost"));

	ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": initialized.");
}

double navigation_cost(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed)
{
	//ROS_INFO_STREAM(__FUNCTION__ << ": parameter count: "<<parameterList.size()<< " relaxed: "<<relaxed);
	TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
	geometry_msgs::Pose2D robot_pose;
	double torso_position = 0.0;
	psu->readRobotPose2D(robot_pose, torso_position, numericalFluentCallback);

	if (relaxed != 0)
	{
		// cost querried by heuristic function. simplified computation: Euclidean distance
		ROS_ASSERT(parameterList.size() == 1);
		const string& table = parameterList[0].value;
		geometry_msgs::Pose table_pose;
		psu->readPose(table_pose, table, numericalFluentCallback);
		double cost_estimate = cost_factor * hypot(table_pose.position.x-robot_pose.x, table_pose.position.y-robot_pose.y) / linear_velocity;
		ROS_INFO_STREAM(__FUNCTION__ << ": estimated cost: "<<cost_estimate);
		return cost_estimate;
	}

	// ([path-condition ?t])
	// should be table grounded_place => param + 1 (due to grounding)
	ROS_ASSERT(parameterList.size() == 2);
	const std::string& grounded_goal = parameterList[1].value;
	nav_msgs::GetPlan srv;
	srv.request.start.header.frame_id = world_frame;
	srv.request.start.pose.position.x = robot_pose.x;
	srv.request.start.pose.position.y = robot_pose.y;
	srv.request.start.pose.position.z = 0.0;
	srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(robot_pose.theta);

	// get the coordinates of (live-grounded) goal pose
	if (! drive_pose::lookup_pose_from_surface_id(grounded_goal, srv.request.goal))
	{
		return INFINITE_COST;
	}
	srv.request.goal.pose.position.z = 0;

	// cache lookup
	string cache_key = create_cache_key(srv.request.start.pose, srv.request.goal.pose);
	double value;
	if (cost_cache->get(cache_key, value))
	{
		ROS_INFO_STREAM(__FUNCTION__ << ": cached cost: "<<value);
		return value;
	}

	// compute value
	ros::WallTime compute_start_time = ros::WallTime::now();
	planning_scene::PlanningScenePtr scene = psu->getCurrentScene(predicateCallback, numericalFluentCallback);
	value = compute_value(scene, srv);
	ros::WallTime compute_end_time = ros::WallTime::now();

	// store in cache
	cost_cache->set(cache_key, value, (compute_end_time - compute_start_time).toSec());
	ROS_INFO_STREAM(__FUNCTION__ << ": computed cost: "<<value);

	return value;
}

int navigation_effect(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
		vector<double> & writtenVars)
{
	// ([path-condition ?t])
	// should be table grounded_place => param + 1 (due to grounding)
	ROS_ASSERT(parameterList.size() == 2);
	const std::string& grounded_goal = parameterList[1].value;
	geometry_msgs::PoseStamped goalPose;
	// get goal from grounding - if not found return 0 (state unchanged)
	if (! drive_pose::lookup_pose_from_surface_id(grounded_goal, goalPose))
		return 0;
	ROS_ASSERT(writtenVars.size() == 4);
	writtenVars[0] = goalPose.pose.position.x;
	writtenVars[1] = goalPose.pose.position.y;
	tf::Quaternion q;
	tf::quaternionMsgToTF(goalPose.pose.orientation, q);
	writtenVars[2] = tf::getYaw(q);
	writtenVars[3] = goalPose.pose.position.z - MIN_TORSO_POSITION;

	return 1;
}
