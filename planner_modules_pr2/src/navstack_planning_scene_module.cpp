#include "planner_modules_pr2/navstack_planning_scene_module.h"
#include "planner_modules_pr2/navstack_module.h"
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include "caching_evaluation.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <utility>
#include <sstream>
using std::map; using std::pair; using std::make_pair; using std::set;
using namespace std;
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

VERIFY_CONDITIONCHECKER_DEF(planning_scene_pathCost);
VERIFY_EXIT_MODULE_DEF(planning_scene_navstack_exit);

string logName = "[psNavModule]";
static CachingEvaluation g_CachingEvaluation;
static const bool g_NavstackCachingEvaluation = true;

//PlanningSceneNavigationModule* PlanningSceneNavigationModule::singleton_instance = NULL;
//
//PlanningSceneNavigationModule* PlanningSceneNavigationModule::instance()
//{
//    if (singleton_instance == NULL) singleton_instance = new PlanningSceneNavigationModule();
//    return singleton_instance;
//}
//
//void PlanningSceneNavigationModule::loadDoorPoses(const string& doorLocationFileName)
//{
//    GeometryPoses locations = GeometryPoses();
//    ROS_ASSERT_MSG(locations.load(doorLocationFileName), "Could not load locations from \"%s\".", doorLocationFileName.c_str());
//    const std::map<std::string, geometry_msgs::PoseStamped>& poses = locations.getPoses();
//    for(std::map<std::string, geometry_msgs::PoseStamped>::const_iterator posesIterator = poses.begin(); posesIterator != poses.end(); posesIterator++)
//    {
//        string name = posesIterator->first;
//        if (StringUtil::startsWith(name, "door"))
//        {
//            string doorName;
//            bool open = false;
//            if(StringUtil::endsWith(name, "_closed"))
//            {
//                doorName = name.substr(0, name.length()-7);
//            }
//            else if(StringUtil::endsWith(name, "_open"))
//            {
//                doorName = name.substr(0, name.length()-5);
//                open = true;
//            }
//            else
//            {
//                ROS_ERROR("navstack planning scene: misformated door location entry %s in file %s", name.c_str(), doorLocationFileName.c_str());
//                continue;
//            }
//            map<string, Door>::iterator doorIterator = doors.find(doorName);
//            if (doorIterator == doors.end())
//            {
//                doors.insert(make_pair(doorName, Door(doorName)));
//                doorIterator = doors.find(doorName);
//            }
//            Door& door = doorIterator->second;
//            if (open)
//            {
//                door.openPose = posesIterator->second;
//            }
//            else
//            {
//                door.closedPose = posesIterator->second;
//            }
//        }
//    }
//}
//
//void PlanningSceneNavigationModule::fillPoseFromState(geometry_msgs::Pose& pose, const string& poseName, numericalFluentCallbackType numericalFluentCallback)
//{
//    // create the numerical fluent request
//    ParameterList startParams;
//    startParams.push_back(Parameter("", "", poseName));
//    NumericalFluentList nfRequest;
//    nfRequest.reserve(7);
//    nfRequest.push_back(NumericalFluent("x", startParams));
//    nfRequest.push_back(NumericalFluent("y", startParams));
//    nfRequest.push_back(NumericalFluent("z", startParams));
//    nfRequest.push_back(NumericalFluent("qx", startParams));
//    nfRequest.push_back(NumericalFluent("qy", startParams));
//    nfRequest.push_back(NumericalFluent("qz", startParams));
//    nfRequest.push_back(NumericalFluent("qw", startParams));
//
//    // get the fluents
//    NumericalFluentList* nfRequestP = &nfRequest;
//    if (!numericalFluentCallback(nfRequestP))
//    {
//        ROS_INFO("fillPoseFromState failed for object: %s", poseName.c_str());
//        return;
//    }
//
//    // fill pose stamped
//    pose.position.x = nfRequest[0].value;
//    pose.position.y = nfRequest[1].value;
//    pose.position.z = nfRequest[2].value;
//    pose.orientation.x = nfRequest[3].value;
//    pose.orientation.y = nfRequest[4].value;
//    pose.orientation.z = nfRequest[5].value;
//    pose.orientation.w = nfRequest[6].value;
//}
//
//bool PlanningSceneNavigationModule::setPlanningSceneDiffFromState(const ParameterList & parameterList,
//        predicateCallbackType predicateCallback,
//        numericalFluentCallbackType numericalFluentCallback)
//{
//
//    // update objects in planning scene
//    PlanningSceneInterface* psi = PlanningSceneInterface::instance();
//    psi->resetPlanningScene();
//    for (map<string, Door>::const_iterator doorIterator = doors.begin(); doorIterator != doors.end(); doorIterator++)
//    {
//        string doorName = doorIterator->first;
//        if (psi->getCollisionObject(doorName) != NULL)
//        {
//            // update door pose
//            PredicateList predicates;
//            ParameterList pl;
//            pl.push_back(Parameter("", "", doorName));
//            predicates.push_back(Predicate("door-open", pl));
//            PredicateList* predicateRequest = &predicates;
//            if ( ! predicateCallback(predicateRequest))
//            {
//                ROS_ERROR("predicateCallback failed for door: %s", doorName.c_str());
//                return false;
//            }
//            geometry_msgs::Pose pose;
//            if (predicates[0].value)
//            {
//                // door is open
//                psi->updateObject(doorName, doorIterator->second.openPose.pose);
//            }
//            else
//            {
//                // door is closed
//                psi->updateObject(doorName, doorIterator->second.closedPose.pose);
//            }
//        }
//    }
//    // set robot state
//    ROS_ASSERT(parameterList.size() > 1);
//    const string& robotStartPose = parameterList[0].value;
//    arm_navigation_msgs::RobotState state = psi->getRobotState();
//    ArmState::get("/arm_configurations/side_tuck/position/", "right_arm").replaceJointPositions(state.joint_state);
//    ArmState::get("/arm_configurations/side_tuck/position/", "left_arm").replaceJointPositions(state.joint_state);
//    fillPoseFromState(state.multi_dof_joint_state.poses[0], robotStartPose, numericalFluentCallback);
//    psi->setRobotState(state);
//    return psi->sendDiff();
//}

void planning_scene_navstack_init(int argc, char** argv)
{
    navstack_init(argc, argv);
    g_CachingEvaluation.setOutputFileName("caching_results_navigation.txt");
    ROS_INFO("%s initialized.", logName.c_str());
}

void planning_scene_navstack_exit(const RawPlan & plan, int argc, char** argv,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback)
{
    ROS_INFO("planning_scene_navstack_exit: writing cache evaluation results");
    g_CachingEvaluation.writeResults();
}

double planning_scene_pathCost(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    // need this for computing cache key
    nav_msgs::GetPlan srv;
    if (!fillPathRequest(parameterList, numericalFluentCallback, srv.request))
    {
        return INFINITE_COST;
    }

    // FIXME do this here for reproducability
    if(relaxed) {
        std::vector<geometry_msgs::PoseStamped> directPath;
        directPath.push_back(srv.request.start);
        directPath.push_back(srv.request.goal);
        return 100 * getPlanCost(directPath); 
    }

    // first lookup in the cache if we answered the query already
    string partialStateCacheKey = computePathCacheKey(parameterList[0].value, parameterList[1].value, srv.request.start.pose, srv.request.goal.pose);
    double cost = INFINITE_COST;
    bool partialWasLocal;
    if(g_PathCostCache.get(partialStateCacheKey, cost, &partialWasLocal, !relaxed))
    {
        // relaxed will be tricky to compare in quality:
        // If we cached well, we'd get the real result before alreayd
        // if caching is bad, we'd not get that and always use the worse relaxed value...
        // But we still shouldn't count this as hit/miss.
        // Best maybe to account for the overhead in computation and
        // only count additional computation time to an absolute reference
        // Can do this, if we show that every caching method hits strictly more!
        if(g_NavstackCachingEvaluation && !relaxed) {
            // partial key cache hit.
            double time;
            if(g_PathCostCache.getTime(partialStateCacheKey, time)) {

                // Check if full state would have hit and record accordingly.
                string fullStateCacheKey = computeFullStateCacheKey(parameterList, predicateCallback, numericalFluentCallback);
                double dummyCost;
                bool fullWasLocal;
                bool fullHit = g_PathCostCache.get(fullStateCacheKey, dummyCost, &fullWasLocal);

                g_CachingEvaluation.recordNoCacheQuery(partialStateCacheKey, time);

                g_CachingEvaluation.recordFullCacheQueryGlobal(partialStateCacheKey, fullHit, time);
                // we're in the partial hit branch
                g_CachingEvaluation.recordPartialCacheQueryGlobal(partialStateCacheKey, true, time);

                g_CachingEvaluation.recordFullCacheQueryLocal(partialStateCacheKey,
                        fullHit && fullWasLocal,
                        time);
                g_CachingEvaluation.recordPartialCacheQueryLocal(partialStateCacheKey, partialWasLocal,
                        time);

                /// Also set the global key - for global eval must be able to determine, if
                /// we already got this full state cached globally.
                g_PathCostCache.set(fullStateCacheKey, cost, true);
            }
        }

        return cost;
    }

    if(relaxed) {
        std::vector<geometry_msgs::PoseStamped> directPath;
        directPath.push_back(srv.request.start);
        directPath.push_back(srv.request.goal);
        return 100 * getPlanCost(directPath); 
    }

    // read state
    string robotLocation = parameterList[0].value;
    geometry_msgs::Pose robotPose;
    map<string, geometry_msgs::Pose> movableObjects;
    GraspedObjectMap graspedObjects;
    map<string, string> objectsOnStatic;
    set<string> openDoors;
//    arm_navigation_msgs::PlanningScene world = PlanningSceneInterface::instance()->getCurrentScene();
    if (! TidyupPlanningSceneUpdater::readState(robotLocation, predicateCallback, numericalFluentCallback, robotPose, movableObjects, graspedObjects, objectsOnStatic))
    {
        ROS_ERROR("%s read state failed.", logName.c_str());
        return INFINITE_COST;
    }
    // set planning scene
    if (! TidyupPlanningSceneUpdater::update(robotPose, movableObjects, graspedObjects))
    {
        ROS_ERROR("%s update planning scene failed.", logName.c_str());
        return INFINITE_COST;
    }
//    PlanningSceneInterface::instance()->printDiffToCurrent(world);

    bool callSuccessful;
    ros::WallTime startCallTime = ros::WallTime::now();
    cost = 10*callPlanningService(srv, parameterList[0].value, parameterList[1].value, callSuccessful);
    if(callSuccessful) {      // only cache real computed paths (including INFINITE_COST)
        ros::WallTime endCallTime = ros::WallTime::now();
        g_PathCostCache.set(partialStateCacheKey, cost, true, (endCallTime - startCallTime).toSec());  // do param cache robot_location calls - they contain the location pose now (safe)

        if(g_NavstackCachingEvaluation) {
            // full miss
            double time = (endCallTime - startCallTime).toSec();
            // Check if full state would have hit and record accordingly.
            string fullStateCacheKey = computeFullStateCacheKey(parameterList, predicateCallback, numericalFluentCallback);
            g_CachingEvaluation.recordNoCacheQuery(partialStateCacheKey, time);
            g_CachingEvaluation.recordFullCacheQueryGlobal(partialStateCacheKey, false, time);
            g_CachingEvaluation.recordPartialCacheQueryGlobal(partialStateCacheKey, false, time);
            g_CachingEvaluation.recordFullCacheQueryLocal(partialStateCacheKey, false, time);
            g_CachingEvaluation.recordPartialCacheQueryLocal(partialStateCacheKey, false, time);
            /// Also set the global key - for global eval must be able to determine, if
            /// we already got this full state cached globally.
            g_PathCostCache.set(fullStateCacheKey, cost, true);
        }
    }

    //cost = 100 * pathCost(parameterList, predicateCallback, numericalFluentCallback, relaxed);
    return cost;
}

