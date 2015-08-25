#include "planner_modules_pr2/drive_pose_module.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tidyup_utils/stringutil.h>
#include <planner_modules_pr2/tidyup_planning_scene_updater.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <symbolic_planning_utils/load_tables.h>
#include <inverse_capability_map/InverseCapabilitySampling.h>

// include for debugging
#include <planner_modules_pr2/EmptyAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>

// debug tools, g_actionDebug enable the triggering of the published msgs
boost::shared_ptr<actionlib::SimpleActionClient<planner_modules_pr2::EmptyAction> > g_action_debug;
/* The DEBUG_PLANNING_SCENE can be used to trigger if the generated planning scene from
 * the symbolic state should be published or not - useful for debugging to see what's
 * going on.
 * To do so, you need to start a action server, using the following command:
 * 		rosrun actionlib axserver.py /empty_action planner_modules_pr2/EmptyAction
 * Each time a new planning scene is generated, it is directly publish to topic: /virtual_planning_scene
 * to see the next ps, just give any feedback using the action server back to the client - not import if
 * a goal is set or a result or feedback (the return value is not checked).
 * If no feedback is provided the timeout is set to 5 minutes, then the next planning scene is published -
 * how this should be enough time to check in rviz.
 */
//#define DEBUG_PLANNING_SCENE
//#define DEBUG_PLANNING_SCENE_INDIVIDUAL_FRAME
// publish the planning scene msgs
ros::Publisher debug_ps_pub_;
// storing the table names with their poses
std::map<std::string, geometry_msgs::PoseStamped> table_poses_;
// storing the table names with their inverse reachability map
std::map<std::string, InverseCapabilityOcTree*> inv_reach_maps_;

// Parameters specifying the sampling with negative update
int number_draws_;
double dev_x_, dev_y_, dev_z_, dev_theta_;
double min_percent_of_max_;
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

// __________________________________________________________________________________________________________________________________________________
void drive_pose_init(int argc, char** argv)
{
	ROS_ASSERT(argc == 1);

    ros::NodeHandle nhPriv("~");
    ros::NodeHandle nh;
    std::string tfPrefix = tf::getPrefixParam(nhPriv);
//    grounding_namespace_ = "grounding/drive_pose_module";

    // first load tables from tables.dat file
	// load table pose
	std::vector<symbolic_planning_utils::LoadTables::TableLocation> tables;
	if (!symbolic_planning_utils::LoadTables::getTables(tables))
	{
		ROS_ERROR("drive_pose_module::%s: Could not load tables!", __func__);
		return;
	}

	// store table names and poses into global variable
	for (size_t i = 0; i < tables.size(); i++)
	{
		const std::string& tableName = tables[i].name;
		std::pair<std::string, geometry_msgs::PoseStamped> tablePose = std::make_pair(tableName, tables[i].pose);
		table_poses_.insert(tablePose);

		// read path to inverse reachability maps from param
		std::string package, relative_path;
		if (!nh.getParam("continual_planning_executive/inverse_reachability_maps/" + tableName + "/package", package))
		{
			ROS_ERROR("drive_pose_module::%s: Could not load package for surface: %s", __func__, tableName.c_str());
			continue;
		}
		if (!nh.getParam("continual_planning_executive/inverse_reachability_maps/" + tableName + "/path", relative_path))
		{
			ROS_ERROR("drive_pose_module::%s: Could not load relative path for surface: %s", __func__, tableName.c_str());
			continue;
		}
		std::string pkg_path = ros::package::getPath(package);
		std::string path = pkg_path + "/" + relative_path;
		// ROS_INFO("path to inv_reach: %s", path.c_str());

		// store inverse reachability maps into global variable
		InverseCapabilityOcTree* tree = InverseCapabilityOcTree::readFile(path);
		std::pair<std::string, InverseCapabilityOcTree*> irm = std::make_pair(tableName, tree);
		inv_reach_maps_.insert(irm);

		// for each table check if there are already some sampled poses
		fetch_poses_from_param(grounding_namespace_, tableName, drive_pose_cache_);
	}

	nhPriv.param("irs/number_draws", number_draws_, 20);
	nhPriv.param("irs/deviation_x", dev_x_, 0.5);
	nhPriv.param("irs/deviation_y", dev_y_, 0.5);
	nhPriv.param("irs/deviation_z", dev_z_, 0.5);
	nhPriv.param("irs/deviation_theta", dev_theta_, M_PI / 4);
	nhPriv.param("irs/min_percent_of_max", min_percent_of_max_, 0.0);

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

#ifdef DEBUG_PLANNING_SCENE
	#ifdef DEBUG_PLANNING_SCENE_INDIVIDUAL_FRAME
		g_action_debug.reset(new actionlib::SimpleActionClient<planner_modules_pr2::EmptyAction>("empty_action", true));
		ROS_INFO("drive_pose_module::%s: Waiting for empty_action action.", __func__);
		ROS_INFO("drive_pose_module::%s: Execute: rosrun actionlib axserver.py /empty_action planner_modules_pr2/EmptyAction", __func__);
		g_action_debug->waitForServer();
	#endif

	std::string ps_topic = "virtual_planning_scene";
	ROS_INFO("drive_pose_module::%s: Debugging of PS enabled, publishing to topic: /%s", __func__, ps_topic.c_str());
	debug_ps_pub_ = nh.advertise<moveit_msgs::PlanningScene>(ps_topic, 1, true);
#endif

   	ROS_INFO_STREAM("drive_pose_module::" << __func__ << ": param namespace: " << nhPriv.getNamespace() << "\n"
   			"number of draws: " << number_draws_ << "\n"
   			"deviation in x: " << dev_x_ << "\n"
   			"deviation in y: " << dev_y_ << "\n"
   			"deviation in z: " << dev_z_ << "\n"
   			"deviation in theta: " << dev_theta_ << "\n"
   			"minimum percent of max: " << min_percent_of_max_ << "\n"
   	);

    ROS_INFO("drive_pose_module::%s: Initialized drive pose Module.", __func__);
}

// __________________________________________________________________________________________________________________________________________________
void drive_pose_exit(const modules::RawPlan & plan, int argc, char** argv,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback)
{
	// empty plan (dummy-plan) is returned by time out
//    if(plan.empty()) {
//        ROS_ERROR("drivePoseExit::%s: failed: No plan produced.", __func__);
//        return;
//    }

    ROS_INFO("drive_pose_module::%s: Putting drive pose cache on param server", __func__);

    // putting drive_pose_cache to param server so actionExec can access
	set_poses_on_param(grounding_namespace_, drive_pose_cache_);

}

// __________________________________________________________________________________________________________________________________________________
std::string determine_drive_pose(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback, modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed, const void* statePtr)
{
	ROS_ASSERT(parameterList.size() == 1);
    std::string table = parameterList[0].value;

///////// HACK
//	if (drive_pose_cache_.size() > 6)
//	{
//		ROS_WARN("Gounded out");
//		return "";
//	}
/////////


	// output of readState function
	geometry_msgs::Pose2D robotPose;
	map<string, geometry_msgs::Pose> movableObjects;
	GraspedObjectMap graspedObjects;
	map<string, string> objectsOnStatic;
	double torsoPosition = 0.0;
	if (!TidyupPlanningSceneUpdater::instance()->readRobotPose2D(robotPose, torsoPosition, numericalFluentCallback))
	{
		ROS_ERROR("drive_pose_module::%s: Could not read robot state from symbolic state!", __func__);
		return "";
	}

	if (!TidyupPlanningSceneUpdater::instance()->readObjects(predicateCallback, numericalFluentCallback, movableObjects, graspedObjects, objectsOnStatic))
	{
		ROS_ERROR("drive_pose_module::%s: Could not read objects from symbolic state!", __func__);
		return "";
	}

	// set planning scene, needed by inv_reach sampling for collision checks
	planning_scene::PlanningScenePtr scene = TidyupPlanningSceneUpdater::instance()->getEmptyScene();
	TidyupPlanningSceneUpdater::instance()->updateRobotPose2D(scene, robotPose, torsoPosition);
	TidyupPlanningSceneUpdater::instance()->updateObjects(scene, movableObjects, graspedObjects);

	std::map<std::string, geometry_msgs::PoseStamped>::iterator it_pose;
	std::map<std::string, InverseCapabilityOcTree*>::iterator it_irm;

	it_pose = table_poses_.find(table);
	it_irm = inv_reach_maps_.find(table);

	if (it_pose == table_poses_.end())
	{
		ROS_ERROR("drive_pose_module::%s: Could not find pose of table: %s", __func__, table.c_str());
		return "";
	}

	if (it_irm == inv_reach_maps_.end())
	{
		ROS_ERROR("drive_pose_module::%s: Could not find inverse reachability map for table: %s", __func__, table.c_str());
		return "";
	}

	InverseCapabilitySampling::PosePercent sampled_pose =
			InverseCapabilitySampling::drawBestOfXSamples(scene, it_irm->second, it_pose->second, number_draws_,
					drive_pose_cache_,
					covariance_,
					min_percent_of_max_);
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


#ifdef DEBUG_PLANNING_SCENE
	// rosrun actionlib axserver.py /empty_action planner_modules_pr2/EmptyAction
	// rosrun actionlib axclient.py /empty_action

    double torsoPosition;
	TidyupPlanningSceneUpdater::instance()->updateRobotPose2D(scene, sampled_pose.pose.pose, torsoPosition);

	moveit_msgs::PlanningScene psMsg;
//	moveit_msgs::PlanningSceneComponents components;
//	components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE |
//			moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
//			moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
//	planning_scene->getPlanningSceneMsg(psMsg, components);
//	ROS_WARN_STREAM(psMsg);

	scene->getPlanningSceneMsg(psMsg);
	ROS_INFO("drive_pose_module::%s: Publishing planning scene message to topic: %s", __func__, debug_ps_pub_.getTopic().c_str());
	debug_ps_pub_.publish(psMsg);

	ros::spinOnce();
	#ifdef DEBUG_PLANNING_SCENE_INDIVIDUAL_FRAME
		ROS_INFO("drive_pose_module::%s: Waiting for user input from Action Server!", __func__);
		planner_modules_pr2::EmptyGoal goal;
		g_action_debug->sendGoal(goal);
		g_action_debug->waitForResult(ros::Duration(5*60));
	#endif
#endif

    ROS_INFO("drive_pose_module::%s: new pose with name: %s", __func__, table_id.c_str());

    return table_id;
}

// __________________________________________________________________________________________________________________________________________________
double robot_near_table(const modules::ParameterList & parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
	ROS_ASSERT(parameterList.size() == 1);
	std::string table = parameterList[0].value;
	double result = modules::INFINITE_COST;

	// fetch surface pose from symbolic state
	geometry_msgs::Pose tablePose;
	TidyupPlanningSceneUpdater::instance()->readPose(tablePose, table, numericalFluentCallback);

	// fetch robot pose from symbolic state
	double torsoPosition;
	geometry_msgs::Pose2D robotPose;
	TidyupPlanningSceneUpdater::instance()->readRobotPose2D(robotPose, torsoPosition, numericalFluentCallback);
	// Update robot pose
	planning_scene::PlanningScenePtr scene = TidyupPlanningSceneUpdater::instance()->getEmptyScene();
	TidyupPlanningSceneUpdater::instance()->updateRobotPose2D(scene, robotPose, torsoPosition);

	// get torso link pose in map frame
	const robot_state::RobotState& robotState = scene->getCurrentState();
	Eigen::Affine3d e = robotState.getGlobalLinkTransform("torso_lift_link");
	tf::Transform transform_map_torso;
	tf::transformEigenToTF(e, transform_map_torso);
//	geometry_msgs::Transform t;
//	tf::transformEigenToMsg(e, t);
//	ROS_WARN_STREAM("Transform: " << t);

	// fetch torso pose and convert into table frame
	tf::Pose transform_map_table, transform_table_torso;
	tf::poseMsgToTF(tablePose, transform_map_table);

	// transform from table to torso (torso pose in table frame)
	transform_table_torso = transform_map_table.inverseTimes(transform_map_torso);

	// fetch matching inverse surface reachability map
	std::map<std::string, InverseCapabilityOcTree*>::iterator it_irm;
	it_irm = inv_reach_maps_.find(table);

	if (it_irm == inv_reach_maps_.end())
	{
		ROS_ERROR("drive_pose_module::%s: Could not find inverse reachability map for table: %s", __func__, table.c_str());
		return modules::INFINITE_COST;
	}
	InverseCapabilityOcTree* irm = it_irm->second;
	// look if there is a match in inv cap map for given torso position in table frame
	InverseCapability inv = irm->getNodeInverseCapability(transform_table_torso.getOrigin().x(),
			transform_table_torso.getOrigin().y(),
			transform_table_torso.getOrigin().z());

	const std::map<double, double>& thetas = inv.getThetasPercent();

	// if there exists a theta, means we have an inverse reachability index,
	// indicating that a part of the table can be reached -> we are close to table and return 0.0 (= true)
	if (thetas.size() > 0)
	{
		ROS_INFO("drive_pose_module::%s: Robot is near '%s' !", __func__, table.c_str());
		result = 0.0;
	}
	else
		result = modules::INFINITE_COST;

	return result;
}
