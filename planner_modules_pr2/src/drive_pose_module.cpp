#include "planner_modules_pr2/drive_pose_module.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tidyup_utils/stringutil.h>
#include <planner_modules_pr2/tidyup_planning_scene_updater.h>

#include <symbolic_planning_utils/load_tables.h>
#include <inverse_capability_map/InverseCapabilitySampling.h>

#include <planner_modules_pr2/EmptyAction.h>
#include <actionlib/client/simple_action_client.h>

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
#define DEBUG_PLANNING_SCENE
ros::Publisher g_debug_ps_pub;
std::map<std::string, geometry_msgs::PoseStamped> g_table_poses;
std::map<std::string, InverseCapabilityOcTree*> g_inv_reach_maps;

int g_inv_reach_sample_draws = 20;

//#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

//VERIFY_CONDITIONCHECKER_DEF(XXX);
//VERIFY_APPLYEFFECT_DEF(XXX);

VERIFY_GROUNDINGMODULE_DEF(determineDrivePose);

// ________________________________________________________________________________________________
void drivePoseInit(int argc, char** argv)
{
	ROS_ASSERT(argc == 1);

    ros::NodeHandle nhPriv("~");
    ros::NodeHandle nh;
    std::string tfPrefix = tf::getPrefixParam(nhPriv);

    // first load tables from tables.dat file
	// load table pose
	std::vector<symbolic_planning_utils::LoadTables::TableLocation> tables;
	if (!symbolic_planning_utils::LoadTables::getTables(tables))
	{
		ROS_ERROR("Could not load tables", __func__);
		return;
	}

	// store table names and poses into global variable
	for (size_t i = 0; i < tables.size(); i++)
	{
		const std::string& tableName = tables[i].name;
		std::pair<std::string, geometry_msgs::PoseStamped> tablePose = std::make_pair(tableName, tables[i].pose);
		g_table_poses.insert(tablePose);

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

		ROS_INFO("path to inv_reach: %s", path.c_str());
		// store inverse reachability maps into global variable
		InverseCapabilityOcTree* tree = InverseCapabilityOcTree::readFile(path);

		std::pair<std::string, InverseCapabilityOcTree*> irm = std::make_pair(tableName, tree);
		g_inv_reach_maps.insert(irm);
	}

	nhPriv.param("inv_reach_sample_draws", g_inv_reach_sample_draws, g_inv_reach_sample_draws);


#ifdef DEBUG_PLANNING_SCENE
    g_action_debug.reset(new actionlib::SimpleActionClient<planner_modules_pr2::EmptyAction>("empty_action", true));
	ROS_INFO("drive_pose_module::%s: Waiting for empty_action action.", __func__);
	ROS_INFO("drive_pose_module::%s: Execute: rosrun actionlib axserver.py /empty_action planner_modules_pr2/EmptyAction", __func__);
	g_action_debug->waitForServer();

	std::string ps_topic = "virtual_planning_scene";
	ROS_INFO("drive_pose_module::%s: Debugging of PS enabled, publishing to topic: /%s", __func__, ps_topic.c_str());
	g_debug_ps_pub = nh.advertise<moveit_msgs::PlanningScene>(ps_topic, 1, true);
#endif

   	ROS_INFO_STREAM("drive_pose_modules::" << __func__ << ": param namespace: " << nhPriv.getNamespace() << "\n"
   			"inv_reach_sample_draws: " << g_inv_reach_sample_draws);

    ROS_INFO("drive_pose_module::%s: Initialized drive pose Module.", __func__);
}

// ________________________________________________________________________________________________
std::string determineDrivePose(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback, modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed, const void* statePtr)
{
    // 1. Get state and request (even if not needed - filter later)
    ROS_ASSERT(parameterList.size() == 2);
    std::string loc = parameterList.at(1).value; // = table1_loc4_room1
    char delim = '_';
    std::vector<std::string> loc_splitted = StringUtil::split(loc, &delim);
    std::string surface = loc_splitted[0]; // = table1

    // input for readState function
    std::string robotLocation = "robot_location";

    ROS_INFO("robot_location: %s", robotLocation.c_str());

	// output of readState function
	geometry_msgs::Pose robotPose;
	std::map<std::string, geometry_msgs::Pose> movableObjects;
	GraspedObjectMap graspedObjects;
	std::map<std::string, std::string> objectsOnStatic;

	TidyupPlanningSceneUpdater::readState(robotLocation, predicateCallback, numericalFluentCallback,
			robotPose, movableObjects, graspedObjects, objectsOnStatic);

	planning_scene::PlanningScenePtr planning_scene;
	TidyupPlanningSceneUpdater::update(robotPose, movableObjects, graspedObjects, planning_scene);


	std::map<std::string, geometry_msgs::PoseStamped>::iterator it_pose;
	std::map<std::string, InverseCapabilityOcTree*>::iterator it_irm;

	it_pose = g_table_poses.find(surface);
	it_irm = g_inv_reach_maps.find(surface);

	if (it_pose == g_table_poses.end())
	{
		ROS_ERROR("drive_pose_module::%s: Could not find pose of table: %s", __func__, surface.c_str());
		return "";
	}

	if (it_irm == g_inv_reach_maps.end())
	{
		ROS_ERROR("drive_pose_module::%s: Could not find inverse reachability map for table: %s", __func__, surface.c_str());
		return "";
	}

	InverseCapabilitySampling::PosePercent sampled_pose = InverseCapabilitySampling::drawBestOfXSamples(planning_scene, it_irm->second, it_pose->second, g_inv_reach_sample_draws);
	ROS_INFO_STREAM("Sampled Pose: Percent: " << sampled_pose.percent << "\n" << sampled_pose.pose);

	robotPose = sampled_pose.pose.pose;
	TidyupPlanningSceneUpdater::update(robotPose, movableObjects, graspedObjects, planning_scene);



#ifdef DEBUG_PLANNING_SCENE
	// rosrun actionlib axserver.py /empty_action planner_modules_pr2/EmptyAction
	// rosrun actionlib axclient.py /empty_action

	moveit_msgs::PlanningScene psMsg;
//	moveit_msgs::PlanningSceneComponents components;
//	components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE |
//			moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
//			moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
//	planning_scene->getPlanningSceneMsg(psMsg, components);
//	ROS_WARN_STREAM(psMsg);

	planning_scene->getPlanningSceneMsg(psMsg);
	ROS_INFO("drive_pose_module::%s: Publishing planning scene message to topic: %s", __func__, g_debug_ps_pub.getTopic().c_str());

	g_debug_ps_pub.publish(psMsg);
	ros::spinOnce();

	ROS_INFO("drive_pose_module::%s: Waiting for user input from Action Server!", __func__);
	planner_modules_pr2::EmptyGoal goal;
	g_action_debug->sendGoal(goal);
	g_action_debug->waitForResult(ros::Duration(5*60));
#endif


    ROS_WARN("drive_pose_module::%s: baseSurface: %s", __func__, surface.c_str());

    return surface;
}

