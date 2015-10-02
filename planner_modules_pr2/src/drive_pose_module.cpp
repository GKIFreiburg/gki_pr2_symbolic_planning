#include "planner_modules_pr2/drive_pose_module.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tidyup_utils/stringutil.h>
#include <planner_modules_pr2/tidyup_planning_scene_updater.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

//#include <symbolic_planning_utils/load_tables.h>
#include "planner_modules_pr2/inverse_reachability_maps.h"
#include <inverse_capability_map/InverseCapabilitySampling.h>

namespace planner_modules_pr2
{

namespace drive_pose
{

// Parameters specifying the sampling with negative update
int number_draws_;
double dev_x_, dev_y_, dev_z_, dev_theta_;
double min_percent_of_max_;
double min_torso_position_;
Eigen::Matrix4d covariance_;

// Cache storing the next free id of a surface
std::map<std::string, int> drive_pose_next_free_cache_;
// Cache storing Grounded Base Pose Name -> Pose
std::map<std::string, geometry_msgs::PoseStamped> drive_pose_cache_;

const std::string grounding_namespace_ =  "grounding/drive_pose_module";

VERIFY_GROUNDINGMODULE_DEF(determine_drive_pose);
VERIFY_EXIT_MODULE_DEF(drive_pose_exit);

// __________________________________________________________________________________________________________________________________________________
bool lookup_pose_from_surface_id(const std::string& surface,
		geometry_msgs::PoseStamped& pose)
{
	std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
	it = drive_pose_cache_.find(surface);
	if (it == drive_pose_cache_.end())
		return false;

	pose = it->second;
	return true;
}

// __________________________________________________________________________________________________________________________________________________
void set_poses_on_param(const std::string& name_space,
		const std::map<std::string, geometry_msgs::PoseStamped>& drive_poses)
{
	std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it;
	for (it = drive_poses.begin(); it != drive_poses.end(); it++)
	{
		// f.ex. /tfd_modules/drive_pose/table1_0/x
		ros::param::set(name_space + "/" + it->first + "/x", it->second.pose.position.x);
		ros::param::set(name_space + "/" + it->first + "/y", it->second.pose.position.y);
		ros::param::set(name_space + "/" + it->first + "/z", it->second.pose.position.z);

		ros::param::set(name_space + "/" + it->first + "/qx", it->second.pose.orientation.x);
		ros::param::set(name_space + "/" + it->first + "/qy", it->second.pose.orientation.y);
		ros::param::set(name_space + "/" + it->first + "/qz", it->second.pose.orientation.z);
		ros::param::set(name_space + "/" + it->first + "/qw", it->second.pose.orientation.w);

		ros::param::set(name_space + "/" + it->first + "/frame_id", it->second.header.frame_id);
	}
}

// __________________________________________________________________________________________________________________________________________________
void fetch_poses_from_param(const std::string& name_space, const std::string& surface,
		std::map<std::string, geometry_msgs::PoseStamped>& drive_poses)
{
	int id = 0;
	geometry_msgs::PoseStamped pose;
	while (true)
	{
		std::stringstream ss;
		ss << id;
		std::string surface_id = surface + "_" + ss.str();
		if (!ros::param::get(name_space + "/" + surface_id + "/x", pose.pose.position.x))
			break;
		if (!ros::param::get(name_space + "/" + surface_id + "/y", pose.pose.position.y))
			break;
		if (!ros::param::get(name_space + "/" + surface_id + "/z", pose.pose.position.z))
			break;
		if (!ros::param::get(name_space + "/" + surface_id + "/qx", pose.pose.orientation.x))
			break;
		if (!ros::param::get(name_space + "/" + surface_id + "/qy", pose.pose.orientation.y))
			break;
		if (!ros::param::get(name_space + "/" + surface_id + "/qz", pose.pose.orientation.z))
			break;
		if (!ros::param::get(name_space + "/" + surface_id + "/qw", pose.pose.orientation.w))
			break;
		if (!ros::param::get(name_space + "/" + surface_id + "/frame_id", pose.header.frame_id))
			break;

		drive_poses[surface_id] = pose;
		id++;
	}
}

} /* namespace drive_pose */

} /* namespace planner_modules_pr2 */

using namespace planner_modules_pr2;
using namespace planner_modules_pr2::drive_pose;
// __________________________________________________________________________________________________________________________________________________
void drive_pose_init(int argc, char** argv)
{
	//ROS_ASSERT(argc == 1);

	planner_modules_pr2::InverseReachabilityMapsPtr icm = planner_modules_pr2::InverseReachabilityMaps::instance();
	for( planner_modules_pr2::InverseReachabilityMap::const_iterator it = icm->begin(); it != icm->end(); it++)
	{
		const string& table_name = it->first;
		fetch_poses_from_param(grounding_namespace_, table_name, drive_pose_cache_);
	}

	ros::NodeHandle nhPriv("~");
	nhPriv.param("irs/number_draws", number_draws_, 20);
	nhPriv.param("irs/deviation_x", dev_x_, 0.5);
	nhPriv.param("irs/deviation_y", dev_y_, 0.5);
	nhPriv.param("irs/deviation_z", dev_z_, 0.5);
	nhPriv.param("irs/deviation_theta", dev_theta_, M_PI / 4);
	nhPriv.param("irs/min_percent_of_max", min_percent_of_max_, 0.0);
	nhPriv.param("irs/min_torso_position", min_torso_position_, 0.012);

	// declare covariances
	double cov_x, cov_y, cov_z, cov_theta;
	cov_x = dev_x_ * dev_x_;
	cov_y = dev_y_ * dev_y_;
	cov_z = dev_z_ * dev_z_;
	cov_theta = dev_theta_ * dev_theta_;
	// declare covariance matrix
	covariance_ << cov_x,  0.0  ,  0.0  , 0.0,
			0.0 , cov_y ,  0.0  , 0.0,
			0.0 ,  0.0  , cov_z , 0.0,
			0.0 ,  0.0  ,  0.0  , cov_theta;

	ROS_INFO_STREAM("drive_pose_module::" << __func__ << ": param namespace: " << nhPriv.getNamespace() << "\n"
			"number of draws: " << number_draws_ << "\n"
			"deviation in x: " << dev_x_ << "\n"
			"deviation in y: " << dev_y_ << "\n"
			"deviation in z: " << dev_z_ << "\n"
			"deviation in theta: " << dev_theta_ << "\n"
			"minimum percent of max: " << min_percent_of_max_ << "\n"
			"minimum torso position: " << min_torso_position_ << "\n"
	);

	ROS_INFO("drive_pose_module::%s: Initialized drive pose Module.", __func__);
}

// __________________________________________________________________________________________________________________________________________________
void drive_pose_exit(const modules::RawPlan & plan, int argc, char** argv,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback)
{
	ROS_INFO("drive_pose_module::%s: Putting drive pose cache on param server", __func__);

	// putting drive_pose_cache to param server so actionExec can access
	set_poses_on_param(grounding_namespace_, drive_pose_cache_);
}

// __________________________________________________________________________________________________________________________________________________
std::string determine_drive_pose(
		const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
		const void* statePtr)
{
	ROS_ASSERT(parameterList.size() == 1);
	std::string table = parameterList[0].value;

	planner_modules_pr2::InverseReachabilityMapsPtr irm = planner_modules_pr2::InverseReachabilityMaps::instance();
	planner_modules_pr2::InverseReachabilityMap::const_iterator it = irm->find(table);
	if (it == irm->end())
	{
		ROS_ERROR("drive_pose_module::%s: Could not find inverse reachability map for table: %s", __func__, table.c_str());
		return "";
	}

	planning_scene::PlanningScenePtr scene = TidyupPlanningSceneUpdater::instance()->getCurrentScene(predicateCallback, numericalFluentCallback);
	InverseCapabilitySampling::PosePercent sampled_pose =
			InverseCapabilitySampling::drawBestOfXSamples(scene, it->second->inverse_reachability.get(), it->second->pose, number_draws_,
					drive_pose_cache_,
					covariance_,
					min_percent_of_max_,
					min_torso_position_);
	ROS_INFO_STREAM("Sampled Pose: Percent: " << sampled_pose.percent << "\n" << sampled_pose.pose);

	int next_free_id = drive_pose_next_free_cache_[table];   // auto inits to 0
	// add next free id to table name
	std::stringstream ss;
	ss << next_free_id;
	std::string table_id = table + "_" + ss.str();
	// ROS_INFO_STREAM("Sampled Pose Id: " << surface_id);

	// a new pose is created - store it in cache
	next_free_id++;
	drive_pose_next_free_cache_[table] = next_free_id;
	drive_pose_cache_[table_id] = sampled_pose.pose;

	ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": new pose with name: "<<table_id);

	return table_id;
}

