#include "planner_modules_pr2/drive_pose_module.h"

#include "planner_modules_pr2/navstack_module.h"
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <map>
using std::map;
#include <utility>
using std::pair; using std::make_pair;
#include <boost/foreach.hpp>
#ifdef __CDT_PARSER__
#define forEach(a, b) for(a : b)
#else
#define forEach BOOST_FOREACH
#endif
#include <sys/times.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"

using namespace modules;


VERIFY_CONDITIONCHECKER_DEF(path_cost);
VERIFY_CONDITIONCHECKER_DEF(path_cost_grounding);
VERIFY_CONDITIONCHECKER_DEF(path_condition_grounding);
VERIFY_APPLYEFFECT_DEF(update_robot_pose);

// Distance measured from ground when torso is at minimum (= not lifted)
const double MIN_TORSO_POSITION = 0.802;

ros::NodeHandle* g_NodeHandle = NULL;
ros::ServiceClient g_GetPlan;

/// Plan requests are issued using this frame - so the poses from the planner are given in this frame (e.g. map)
std::string g_WorldFrame;

double g_GoalTolerance = 0.5;

double g_TransSpeed = 0.3;
double g_RotSpeed = angles::from_degrees(30);

// Using a cache of queried path costs to prevent calling the path planning service multiple times
// Better: Can we assume symmetric path costs?
//map< pair<string,string>, double> g_PathCostCache;
ModuleParamCacheDouble g_PathCostCache;
string compute_path_cache_key(const string& startLocation, const string& goalLocation,
        const geometry_msgs::Pose & startPose, const geometry_msgs::Pose & goalPose)
{
    std::string startPoseStr = createPoseParamString(startPose, 0.01, 0.02);
    std::string goalPoseStr = createPoseParamString(goalPose, 0.01, 0.02);

    if (startLocation < goalLocation)
    {
        return startLocation + "_" + startPoseStr + "__" + goalLocation + "_" + goalPoseStr;
    }
    else
    {
        return goalLocation + "_" + goalPoseStr + "__" + startLocation + "_" + startPoseStr;
    }
}

void navstack_init(int argc, char** argv)
{
    ROS_ASSERT(argc >= 4);

    // get world frame
    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);
    g_WorldFrame = tf::resolve(tfPrefix, argv[1]);
    ROS_INFO("World frame is: %s", g_WorldFrame.c_str());

    nhPriv.param("trans_speed", g_TransSpeed, g_TransSpeed);
    nhPriv.param("rot_speed", g_RotSpeed, g_RotSpeed);

    // get goal tolerance
    char* checkPtr;
    g_GoalTolerance = strtod(argv[2], &checkPtr);
    if(checkPtr == argv[2]) {    // conversion error!
        ROS_ERROR("%s: Could not convert argument for goal tolerance: %s", __func__, argv[2]);
        g_GoalTolerance = 0.5;
    }

    ros::NodeHandle nh;
    std::string base_local_planner_ns;
    if(strcmp(argv[3], "0") == 0) {
        ROS_INFO("Using absolute goal tolerance.");
    } else if(strcmp(argv[3], "1") == 0) {
        ROS_INFO("Trying to estimate base_local_planner namespace");

        std::string local_planner;
        if(!nh.getParam("move_base_node/base_local_planner", local_planner)
                && !nh.getParam("move_base/base_local_planner", local_planner)) {
            ROS_ERROR("move_base(_node)/base_local_planner not set - falling back to absolute mode.");
        } else {
            // dwa_local_planner/DWAPlannerROS -> DWAPlannerROS
            std::string::size_type x = local_planner.find_last_of("/");
            if(x == std::string::npos)
                base_local_planner_ns = local_planner;
            else
                base_local_planner_ns = local_planner.substr(x + 1);
            // add move_base(_node) prefix
            string dummy;
            if(nh.getParam("move_base_node/base_local_planner", dummy)) {
                base_local_planner_ns = "move_base_node/" + base_local_planner_ns;
            } else if(nh.getParam("move_base/base_local_planner", dummy)) {
                base_local_planner_ns = "move_base/" + base_local_planner_ns;
            } else {
                ROS_ASSERT(false);
            }

            ROS_INFO("Estimated base_local_planner_ns to %s.", base_local_planner_ns.c_str());
        }
    } else {
        base_local_planner_ns = argv[3];
    }

    if(!base_local_planner_ns.empty()) {
        // relative goal tolerance, argv[3] contains the base_local_planner namespace
        ROS_INFO("Using relative goal tolerance.");
        // get base_local_planner's xy_goal_tolerance
        double move_base_tol;
        if(!nh.getParam(base_local_planner_ns + "/xy_goal_tolerance", move_base_tol)) {
            ROS_ERROR_STREAM("requested relative goal tolerance, but "
                    << (base_local_planner_ns + "/xy_goal_tolerance") << " was not set"
                    << " - falling back to absolute mode");
        } else { // 2. add move_base's tolerance to our relative tolerance
            g_GoalTolerance += move_base_tol;
        }
    }

    ROS_INFO("Goal Tolerance is: %f.", g_GoalTolerance);

    // init service query for make plan
    string service_name = "move_base_node/make_plan";
    g_NodeHandle = new ros::NodeHandle();
//TODO:
//    while(!ros::service::waitForService(service_name, ros::Duration(3.0))) {
//        ROS_ERROR("Service %s not available - waiting.", service_name.c_str());
//    }

    g_GetPlan = g_NodeHandle->serviceClient<nav_msgs::GetPlan>(service_name, true);
    if(!g_GetPlan) {
        ROS_FATAL("Could not initialize get plan service from %s (client name: %s)", service_name.c_str(), g_GetPlan.getService().c_str());
    }
    ROS_INFO("Service connection to %s established.", g_GetPlan.getService().c_str());

    g_PathCostCache.initialize("move_base", g_NodeHandle);

    ROS_INFO("Initialized Navstack Module.");
}

bool fill_path_request(const ParameterList & parameterList, numericalFluentCallbackType numericalFluentCallback,
        nav_msgs::GetPlan::Request & request)
{
    // get robot and target location from planner interface
    ROS_ASSERT(parameterList.size() == 2);

    ParameterList startParams;
    startParams.push_back(parameterList[0]);
    ParameterList goalParams;
    goalParams.push_back(parameterList[1]);
    NumericalFluentList nfRequest;
    nfRequest.reserve(14);
    nfRequest.push_back(NumericalFluent("x", startParams));
    nfRequest.push_back(NumericalFluent("y", startParams));
    nfRequest.push_back(NumericalFluent("z", startParams));
    nfRequest.push_back(NumericalFluent("qx", startParams));
    nfRequest.push_back(NumericalFluent("qy", startParams));
    nfRequest.push_back(NumericalFluent("qz", startParams));
    nfRequest.push_back(NumericalFluent("qw", startParams));
    nfRequest.push_back(NumericalFluent("x", goalParams));
    nfRequest.push_back(NumericalFluent("y", goalParams));
    nfRequest.push_back(NumericalFluent("z", goalParams));
    nfRequest.push_back(NumericalFluent("qx", goalParams));
    nfRequest.push_back(NumericalFluent("qy", goalParams));
    nfRequest.push_back(NumericalFluent("qz", goalParams));
    nfRequest.push_back(NumericalFluent("qw", goalParams));

    NumericalFluentList* nfRequestP = &nfRequest;
    if(!numericalFluentCallback(nfRequestP)) {
        ROS_ERROR("numericalFluentCallback failed.");
        return false;
    }

    // create the path planning query for service
    request.start.header.frame_id = g_WorldFrame;
    request.goal.header.frame_id = g_WorldFrame;
    request.start.pose.position.x = nfRequest[0].value;
    request.start.pose.position.y = nfRequest[1].value;
    request.start.pose.position.z = nfRequest[2].value;
    request.start.pose.orientation.x = nfRequest[3].value;
    request.start.pose.orientation.y = nfRequest[4].value;
    request.start.pose.orientation.z = nfRequest[5].value;
    request.start.pose.orientation.w = nfRequest[6].value;
    request.goal.pose.position.x = nfRequest[7].value;
    request.goal.pose.position.y = nfRequest[8].value;
    request.goal.pose.position.z = nfRequest[9].value;
    request.goal.pose.orientation.x = nfRequest[10].value;
    request.goal.pose.orientation.y = nfRequest[11].value;
    request.goal.pose.orientation.z = nfRequest[12].value;
    request.goal.pose.orientation.w = nfRequest[13].value;
    request.tolerance = g_GoalTolerance;
    return true;
}

double get_plan_cost(const std::vector<geometry_msgs::PoseStamped> & plan)
{
    if(plan.empty())
        return 0;

    double pathLength = 0;
    double rotLength = 0;
    geometry_msgs::PoseStamped lastPose = plan[0];
    forEach(const geometry_msgs::PoseStamped & p, plan) {
        double d = hypot(lastPose.pose.position.x - p.pose.position.x,
                lastPose.pose.position.y - p.pose.position.y);
        pathLength += d;

        double yawCur = tf::getYaw(p.pose.orientation);
        double yawLast = tf::getYaw(lastPose.pose.orientation);
        double da = fabs(angles::normalize_angle(yawCur - yawLast));
        rotLength += da;

        lastPose = p;
    }
    return pathLength/g_TransSpeed + rotLength/g_RotSpeed;
}

double call_planning_service(nav_msgs::GetPlan& srv, const string& startLocationName, const string& goalLocationName,
        bool & callSuccessful)
{
    callSuccessful = false;
    double cost = INFINITE_COST;
    if (!g_GetPlan)
    {
        ROS_ERROR("GetPlan Persistent service connection to %s failed.", g_GetPlan.getService().c_str());
        // FIXME reconnect - this shouldn't happen.
        return INFINITE_COST;
    }

    // statistics about using the ros path planner service
    static double plannerCalls = 0;
    static ros::WallDuration totalCallsTime = ros::WallDuration(0.0);
    plannerCalls += 1.0;

    ros::WallTime callStartTime = ros::WallTime::now();
    // This construct is here, because when the robot is moving move_base will not produce other paths
    // we retry for a certain amount of time to not fail directly.
    ROS_INFO_STREAM("planner call: start " << startLocationName << " (" << srv.request.start.pose.position
            << "). goal " << goalLocationName << " (" << srv.request.goal.pose.position << ")");
    static unsigned int failCounter = 0;
    ros::Rate retryRate = 1;
    do
    {
        ROS_INFO("%s: Calling path planner.", __func__);
        // perform the actual path planner call
        if(g_GetPlan.call(srv))
        {
            failCounter = 0;    // will also exit loop
            callSuccessful = true;

            if (!srv.response.plan.poses.empty())
            {
                // get plan cost
                cost = get_plan_cost(srv.response.plan.poses);
                ROS_DEBUG("Got plan: %s -> %s cost: %f.", startLocationName.c_str(), goalLocationName.c_str(), cost);
            }
            else    // no plan found.
            {
                ROS_WARN("Got empty plan: %s -> %s", startLocationName.c_str(), goalLocationName.c_str());
                cost = INFINITE_COST;
            }

            ros::WallTime callEndTime = ros::WallTime::now();
            ros::WallDuration dt = callEndTime - callStartTime;
            totalCallsTime += dt;
            ROS_DEBUG("ServiceCall took: %f s, avg: %f s (num %f).",
                    dt.toSec(), totalCallsTime.toSec()/plannerCalls, plannerCalls);
        }
        else
        {
            ROS_ERROR("Failed to call service %s - is the robot moving?", g_GetPlan.getService().c_str());
            failCounter++;
            retryRate.sleep();
        }
    } while (failCounter < 300 && failCounter > 0);

    return cost;
}

double path_cost(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    if (g_Debug)
    { // prevent spamming ROS_DEBUG calls unless we really want debug
        // debugging raw planner calls
        static unsigned long calls = 0;
        calls++;
        ROS_DEBUG_THROTTLE(1.0, "Got %lu module calls.\n", calls);
    }
    ROS_ASSERT(parameterList.size() == 2);

    nav_msgs::GetPlan srv;
    if (!fill_path_request(parameterList, numericalFluentCallback, srv.request))
    {
        return INFINITE_COST;
    }

    // first lookup in the cache if we answered the query already
    double cost = INFINITE_COST;
    string cacheKey = compute_path_cache_key(parameterList[0].value, parameterList[1].value, srv.request.start.pose, srv.request.goal.pose);
    if (g_PathCostCache.get(cacheKey, cost))
    {
        return cost;
    }

    bool callSuccessful;
    ros::WallTime startCallTime = ros::WallTime::now();
    cost = call_planning_service(srv, parameterList[0].value, parameterList[1].value, callSuccessful);
    if(callSuccessful) {
        // only cache real computed paths (including INFINITE_COST)
        //bool isRobotLocation =
        //    (parameterList[0].value == "robot_location" || parameterList[1].value == "robot_location");
        //g_PathCostCache.set(cacheKey, cost, !isRobotLocation);  // do no param cache robot_location calls
        ros::WallTime endCallTime = ros::WallTime::now();
        g_PathCostCache.set(cacheKey, cost, true, (endCallTime - startCallTime).toSec());  // do param cache robot_location calls - they contain the location pose now (safe)
    }
    return cost;
}

double path_cost_grounding(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback,
		int relaxed)
{
	if (g_Debug)
	{ // prevent spamming ROS_DEBUG calls unless we really want debug
	  // debugging raw planner calls
		static unsigned long calls = 0;
		calls++;
		ROS_DEBUG_THROTTLE(1.0, "Got %lu module calls.\n", calls);
	}
	ROS_INFO_STREAM("navstack_module::" << __func__ << ": parameter count: "<<parameterList.size());
	for (size_t i = 0; i < parameterList.size(); i++)
	{
		ROS_INFO_STREAM(parameterList[i].value);
	}

	ROS_ASSERT(parameterList.size() == 2);
	const std::string& grounded_goal = parameterList[1].value;

	nav_msgs::GetPlan srv;
	if (! fill_robot_pose_XYT(numericalFluentCallback, srv.request.start))
	{
		return INFINITE_COST;
	}
	// get goal from grounding - if not found return inf cost
	if (!lookup_pose_from_surface_id(grounded_goal, srv.request.goal))
		return INFINITE_COST;

	// first lookup in the cache if we answered the query already
	double cost = INFINITE_COST;
	string cacheKey = compute_path_cache_key("robot_location", parameterList[0].value, srv.request.start.pose, srv.request.goal.pose);
	if (g_PathCostCache.get(cacheKey, cost))
	{
		return cost;
	}

	bool callSuccessful;
	ros::WallTime startCallTime = ros::WallTime::now();
	cost = call_planning_service(srv, "robot_location", parameterList[0].value, callSuccessful);
	if (callSuccessful)
	{      // only cache real computed paths (including INFINITE_COST)
		//bool isRobotLocation =
		//    (parameterList[0].value == "robot_location" || parameterList[1].value == "robot_location");
		//g_PathCostCache.set(cacheKey, cost, !isRobotLocation);  // do no param cache robot_location calls
		ros::WallTime endCallTime = ros::WallTime::now();
		g_PathCostCache.set(cacheKey, cost, true, (endCallTime - startCallTime).toSec());  // do param cache robot_location calls - they contain the location pose now (safe)
	}
	return cost;
}

double path_condition_grounding(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback,
		int relaxed)
{
	if (g_Debug)
	{ // prevent spamming ROS_DEBUG calls unless we really want debug
	  // debugging raw planner calls
		static unsigned long calls = 0;
		calls++;
		ROS_DEBUG_THROTTLE(1.0, "Got %lu module calls.\n", calls);
	}
	ROS_INFO_STREAM("navstack_module::" << __func__ << ": parameter count: "<<parameterList.size());
	for (size_t i = 0; i < parameterList.size(); i++)
	{
		ROS_INFO_STREAM(parameterList[i].value);
	}

	// ([path-condition ?t])
	// should be table grounded_place => param + 1 (due to grounding)
	ROS_ASSERT(parameterList.size() == 2);
	const std::string& grounded_goal = parameterList[1].value;

	geometry_msgs::Pose2D robot_pose;
	double torso_position;
	TidyupPlanningSceneUpdater* updater = TidyupPlanningSceneUpdater::instance();
	updater->readRobotPose2D(robot_pose, torso_position, numericalFluentCallback);
	planning_scene::PlanningScenePtr scene = updater->getEmptyScene();
	updater->updateRobotPose2D(scene, robot_pose, torso_position);
	updater->visualize(scene);


	nav_msgs::GetPlan srv;
	if (! fill_robot_pose_XYT(numericalFluentCallback, srv.request.start))
	{
		return INFINITE_COST;
	}

	// get goal from grounding - if not found return inf cost
	if (!lookup_pose_from_surface_id(grounded_goal, srv.request.goal))
		return INFINITE_COST;
	srv.request.goal.pose.position.z = 0;

	// first lookup in the cache if we answered the query already
	double cost = INFINITE_COST;
	string cacheKey = compute_path_cache_key("robot_location", grounded_goal, srv.request.start.pose, srv.request.goal.pose);
	if (g_PathCostCache.get(cacheKey, cost))
	{
		return cost;
	}

	bool callSuccessful;
	ros::WallTime startCallTime = ros::WallTime::now();
	cost = call_planning_service(srv, "robot_location", grounded_goal, callSuccessful);
	if (callSuccessful)
	{      // only cache real computed paths (including INFINITE_COST)
		//bool isRobotLocation =
		//    (parameterList[0].value == "robot_location" || parameterList[1].value == "robot_location");
		//g_PathCostCache.set(cacheKey, cost, !isRobotLocation);  // do no param cache robot_location calls
		ros::WallTime endCallTime = ros::WallTime::now();
		g_PathCostCache.set(cacheKey, cost, true, (endCallTime - startCallTime).toSec());  // do param cache robot_location calls - they contain the location pose now (safe)
	}
	return cost;
}

int update_robot_pose(
		const modules::ParameterList& parameterList,
		modules::predicateCallbackType predicateCallback,
		modules::numericalFluentCallbackType numericalFluentCallback,
		int relaxed,
		vector<double> & writtenVars)
{
	ROS_INFO_STREAM("navstack_module::" << __func__ << ": parameter count: "<<parameterList.size());
	for (size_t i = 0; i < parameterList.size(); i++)
	{
		ROS_INFO_STREAM(parameterList[i].value);
	}

	// ([path-condition ?t])
	// should be table grounded_place => param + 1 (due to grounding)
	ROS_ASSERT(parameterList.size() == 2);
	const std::string& grounded_goal = parameterList[1].value;
	geometry_msgs::PoseStamped goalPose;
	// get goal from grounding - if not found return 0 (state unchanged)
	if (!lookup_pose_from_surface_id(grounded_goal, goalPose))
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

bool fill_robot_pose_XYT(
		modules::numericalFluentCallbackType numericalFluentCallback,
		geometry_msgs::PoseStamped& robot_pose)
{
	ParameterList poseParams;
	NumericalFluentList nfRequest;
	nfRequest.reserve(3);
	nfRequest.push_back(NumericalFluent("robot-x", poseParams));
	nfRequest.push_back(NumericalFluent("robot-y", poseParams));
	nfRequest.push_back(NumericalFluent("robot-theta", poseParams));

	NumericalFluentList* nfRequestP = &nfRequest;
	if ( !numericalFluentCallback(nfRequestP))
	{
		ROS_ERROR("numericalFluentCallback failed.");
		return false;
	}

	robot_pose.header.frame_id = g_WorldFrame;
	robot_pose.pose.position.x = nfRequest[0].value;
	robot_pose.pose.position.y = nfRequest[1].value;
	robot_pose.pose.position.z = 0.0;
	robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(nfRequest[2].value);
	return true;
}

