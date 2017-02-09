#include "planner_modules_pr2/drive_pose_module.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tidyup_utils/stringutil.h>
#include <planner_modules_pr2/tidyup_planning_scene_updater.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>

//#include <symbolic_planning_utils/load_tables.h>
#include <planner_modules_pr2/module_param_cache.h>
#include "planner_modules_pr2/inverse_reachability_maps.h"
#include <inverse_capability_map/InverseCapabilitySampling.h>

namespace planner_modules_pr2
{

namespace drive_pose
{
using std::string;
using std::vector;
using std::map;
using std::stringstream;
using std::pair;

void convert_pose(const geometry_msgs::Pose& in, geometry_msgs::Pose2D& goal_pose)
{
  goal_pose.x = in.position.x;
  goal_pose.y = in.position.y;
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(in.orientation, tf_q);
  goal_pose.theta = tf::getYaw(tf_q);
}

void convert_pose(const geometry_msgs::Pose2D& in, geometry_msgs::Pose& out)
{
  out.position.x = in.x;
  out.position.y = in.y;
  out.position.z = 0;
  out.orientation = tf::createQuaternionMsgFromYaw(in.theta);
}


// module param cache, to track sampling wall time and quality
boost::shared_ptr<ModuleParamCache<string> > pose_name_cache;
string create_cache_key(const geometry_msgs::Pose2D& pose)
{
  return createPoseParamString(pose, 0.0, 0.2, 0.2);
}

// Parameters specifying the sampling with negative update
int number_draws_;
double dev_x_, dev_y_, dev_z_, dev_theta_;
double min_percent_of_max_;
double min_torso_position_;
Eigen::Matrix4d covariance_;

typedef map<string, int> TablePoseCountMap;
typedef pair<const void*, string> StateTablePair;
typedef map<StateTablePair, int> StateRequestCountMap;

StateRequestCountMap state_requests;
TablePoseCountMap table_pose_counters;

// Cache storing Grounded Base Pose Name -> Pose
map<string, geometry_msgs::PoseStamped> sampled_poses;

const string grounding_namespace_ = "grounding/drive_pose_module";

VERIFY_GROUNDINGMODULE_DEF (determine_drive_pose);
VERIFY_EXIT_MODULE_DEF (drive_pose_exit);

// __________________________________________________________________________________________________________________________________________________
bool lookup_pose_from_surface_id(const string& surface, geometry_msgs::PoseStamped& pose)
{
  map<string, geometry_msgs::PoseStamped>::iterator it;
  it = sampled_poses.find(surface);
  if (it == sampled_poses.end())
    return false;

  pose = it->second;
  return true;
}

// __________________________________________________________________________________________________________________________________________________
void set_poses_on_param(const string& name_space,
    const map<string, geometry_msgs::PoseStamped>& drive_poses)
{
  map<string, geometry_msgs::PoseStamped>::const_iterator it;
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

string construct_pose_name(const string& table, int id)
{
  stringstream ss;
  ss << table << "_" << id;
  return ss.str();
}

// __________________________________________________________________________________________________________________________________________________
void fetch_poses_from_param(const string& name_space, const string& table,
    map<string, geometry_msgs::PoseStamped>& drive_poses)
{
  int& id = table_pose_counters[table];
  geometry_msgs::PoseStamped pose;
  while (true)
  {
    string pose_name = construct_pose_name(table, id);
    if (!ros::param::get(name_space + "/" + pose_name + "/x", pose.pose.position.x))
      break;
    if (!ros::param::get(name_space + "/" + pose_name + "/y", pose.pose.position.y))
      break;
    if (!ros::param::get(name_space + "/" + pose_name + "/z", pose.pose.position.z))
      break;
    if (!ros::param::get(name_space + "/" + pose_name + "/qx", pose.pose.orientation.x))
      break;
    if (!ros::param::get(name_space + "/" + pose_name + "/qy", pose.pose.orientation.y))
      break;
    if (!ros::param::get(name_space + "/" + pose_name + "/qz", pose.pose.orientation.z))
      break;
    if (!ros::param::get(name_space + "/" + pose_name + "/qw", pose.pose.orientation.w))
      break;
    if (!ros::param::get(name_space + "/" + pose_name + "/frame_id", pose.header.frame_id))
      break;

    drive_poses[pose_name] = pose;
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
  for (planner_modules_pr2::InverseReachabilityMap::const_iterator it = icm->begin(); it != icm->end(); it++)
  {
    const string& table_name = it->first;
    fetch_poses_from_param(grounding_namespace_, table_name, sampled_poses);
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
  covariance_ << cov_x, 0.0, 0.0, 0.0, 0.0, cov_y, 0.0, 0.0, 0.0, 0.0, cov_z, 0.0, 0.0, 0.0, 0.0, cov_theta;

  ROS_INFO_STREAM(
      "drive_pose_module::" << __func__ << ": param namespace: "
      << nhPriv.getNamespace() << "\n" "number of draws: " << number_draws_
      << "\n" "deviation in x: " << dev_x_ << "\n" "deviation in y: " << dev_y_
      << "\n" "deviation in z: " << dev_z_ << "\n" "deviation in theta: " << dev_theta_
      << "\n" "minimum percent of max: " << min_percent_of_max_ << "\n" "minimum torso position: "
      << min_torso_position_ << "\n");

  pose_name_cache.reset(new ModuleParamCache<string>("robot_pose/grounding"));
  ROS_INFO("drive_pose_module::%s: Initialized drive pose Module.", __func__);
}

// __________________________________________________________________________________________________________________________________________________
void drive_pose_exit(const modules::RawPlan & plan, int argc, char** argv,
    modules::predicateCallbackType predicateCallback, modules::numericalFluentCallbackType numericalFluentCallback)
{
  ROS_INFO("drive_pose_module::%s: Putting drive pose cache on param server", __func__);

  // putting drive_pose_cache to param server so actionExec can access
  set_poses_on_param(grounding_namespace_, sampled_poses);
}

// __________________________________________________________________________________________________________________________________________________
string determine_drive_pose(const modules::ParameterList & parameterList,
    modules::predicateCallbackType predicateCallback, modules::numericalFluentCallbackType numericalFluentCallback,
    int relaxed, const void* statePtr)
{
  ROS_ASSERT(parameterList.size() == 1);
  string table = parameterList[0].value;

  // only sample new poses, if *that* state requested more than we have sapled previously.
  int& state_request_counter = state_requests[std::make_pair(statePtr, table)];
  state_request_counter++;
  int& table_pose_counter = table_pose_counters[table];
  if (state_request_counter <= table_pose_counter)
  {
    // return one of the poses already sampled.
    // poses are zero-indexed
    string pose_name = construct_pose_name(table, state_request_counter-1);
    ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": existing pose: "<<pose_name);
    return pose_name;
  }

  planner_modules_pr2::InverseReachabilityMapsPtr irm = planner_modules_pr2::InverseReachabilityMaps::instance();
  planner_modules_pr2::InverseReachabilityMap::const_iterator it = irm->find(table);
  if (it == irm->end())
  {
    ROS_ERROR("drive_pose_module::%s: Could not find inverse reachability map for table: %s", __func__, table.c_str());
    return "";
  }

  ros::WallTime sampling_time_start = ros::WallTime::now();
  planning_scene::PlanningScenePtr scene = TidyupPlanningSceneUpdater::instance()->getCurrentScene(predicateCallback,
      numericalFluentCallback);
  InverseCapabilitySampling::PosePercent sampled_pose = InverseCapabilitySampling::drawBestOfXSamples(scene,
      it->second->inverse_reachability.get(), it->second->pose, number_draws_, sampled_poses, covariance_,
      min_percent_of_max_, min_torso_position_);
  ros::WallTime sampling_time_finish = ros::WallTime::now();

  string pose_name = construct_pose_name(table, table_pose_counter);
  sampled_poses[pose_name] = sampled_pose.pose;
  table_pose_counter++;

  geometry_msgs::Pose2D pose;
  convert_pose(sampled_pose.pose.pose, pose);
  string cache_key = create_cache_key(pose);
  pose_name_cache->set(cache_key, pose_name, (sampling_time_finish - sampling_time_start).toSec());
  ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": new pose: "<<pose_name);
  ROS_DEBUG_STREAM("Sampled Pose: Percent: " << sampled_pose.percent << "\n" << sampled_pose.pose);


  return pose_name;
}

