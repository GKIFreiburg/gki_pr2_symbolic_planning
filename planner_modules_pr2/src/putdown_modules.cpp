#include "planner_modules_pr2/putdown_modules.h"
#include "planner_modules_pr2/module_param_cache.h"
//#include "tidyup_utils/arm_state.h"
//#include "tidyup_utils/planning_scene_interface.h"
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include "tidyup_utils/planning_scene_interface.h"
//#include "tidyup_utils/transformer.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <set>
using std::set;
using std::map;
#include <utility>
using std::pair; using std::make_pair;
//#include <boost/foreach.hpp>
//#ifdef __CDT_PARSER__
//#define forEach(a, b) for(a : b)
//#else
//#define forEach BOOST_FOREACH
//#endif
#include <sys/times.h>
//#include <tf/tf.h>
//#include <tf/transform_listener.h>
#include <moveit_msgs/MoveItErrorCodes.h>
//#include <moveit_msgs/convert_messages.h>
//#include <boost/tuple/tuple.hpp>
//#include <boost/tuple/tuple_comparison.hpp>
#include <tidyup_msgs/GetPutdownPose.h>
#include <sstream>
#include "tidyup_utils/stringutil.h"
#include "subsumption_caching.h"
#include "caching_evaluation.h"

VERIFY_CONDITIONCHECKER_DEF(canPutdown);
VERIFY_APPLYEFFECT_DEF(updatePutdownPose);

string g_WorldFrame;
ros::NodeHandle* g_NodeHandle = NULL;
ros::ServiceClient g_GetPutdownPose;

ModuleParamCacheString paramCache;
static SubsumptionCache* g_SubsumptionCache = NULL;
static SubsumptionCache* g_SubsumptionCacheGlobal = NULL;
string separator = " ";

string logName;
geometry_msgs::Pose defaultAttachPose;

static CachingEvaluation g_PutdownCachingEvaluation;
static const bool g_EvaluatePutdownCaching = true;

void putdown_init(int argc, char** argv)
{
    ROS_ASSERT(argc == 2);
    logName = "[putdownModule]";

    g_NodeHandle = new ros::NodeHandle();
    string service_name = "/tidyup/request_putdown_pose";

// TODO:
//    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
//    {
//        ROS_ERROR("%s Service %s not available - waiting.", logName.c_str(), service_name.c_str());
//    }

    g_GetPutdownPose = g_NodeHandle->serviceClient<tidyup_msgs::GetPutdownPose>(service_name, true);
    if (!g_GetPutdownPose)
    {
        ROS_FATAL("%s Could not initialize get putdown service from %s (client name: %s)", logName.c_str(), service_name.c_str(), g_GetPutdownPose.getService().c_str());
    }

    // initialize cache
    paramCache.initialize("putdown", g_NodeHandle);

    defaultAttachPose.position.x = 0.032;
    defaultAttachPose.position.y = 0.015;
    defaultAttachPose.position.z = 0.0;
    defaultAttachPose.orientation.x = 0.707;
    defaultAttachPose.orientation.y = -0.106;
    defaultAttachPose.orientation.z = -0.690;
    defaultAttachPose.orientation.w = 0.105;

    g_SubsumptionCache = new SubsumptionCache();
    g_SubsumptionCacheGlobal = new SubsumptionCache("putdown_subsumption");

    if(!g_SubsumptionCacheGlobal->readFromParams()) {
        ROS_ERROR("Failed to read data for subsumption cache.");
    } else {
        ROS_INFO("Read data for subsumption cache.");
    }

    g_PutdownCachingEvaluation.setOutputFileName("caching_results_putdown.txt");

    ROS_INFO("putdown_init cached");
    //ROS_INFO("ModuleParamCache local");
    //paramCache.dump();
    ROS_INFO("g_SubsumptionCache");
    g_SubsumptionCache->dump();
    ROS_INFO("g_SubsumptionCacheGlobal");
    g_SubsumptionCacheGlobal->dump();

    ROS_INFO("%s Initialized Putdown Module.\n", logName.c_str());
}

void putdown_exit(const RawPlan & plan, int argc, char** argv,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback)
{
    ROS_INFO("putdown_exit: writing subsumption cache to params");
    g_SubsumptionCacheGlobal->writeToParams();

    ROS_INFO("putdown_exit: writing cache evaluation results");
    g_PutdownCachingEvaluation.writeResults();

    //ROS_INFO("ModuleParamCache local");
    //paramCache.dump();
    ROS_INFO("g_SubsumptionCache");
    g_SubsumptionCache->dump();
    ROS_INFO("g_SubsumptionCacheGlobal");
    g_SubsumptionCacheGlobal->dump();
}

bool callFindPutdownPoseService(tidyup_msgs::GetPutdownPose & srv)
{
    if (!g_GetPutdownPose)
    {
        ROS_ERROR("%s Persistent service connection to %s failed.", logName.c_str(), g_GetPutdownPose.getService().c_str());
        // FIXME reconnect - this shouldn't happen.
        return false;
    }

    // statistics about using the ros service
    static double plannerCalls = 0;
    static ros::Duration totalCallsTime = ros::Duration(0.0);
    plannerCalls += 1.0;
    ros::Time callStartTime = ros::Time::now();

    // perform the actual path planner call
    if (! g_GetPutdownPose.call(srv))
    {
        ROS_ERROR("%s Failed to call service %s.", logName.c_str(), g_GetPutdownPose.getService().c_str());
        return false;
    }
    if (g_Debug)
    {
        ros::Time callEndTime = ros::Time::now();
        ros::Duration dt = callEndTime - callStartTime;
        totalCallsTime += dt;
        ROS_DEBUG("%s ServiceCall took: %f, avg: %f (num %f).", logName.c_str(),
                dt.toSec(), totalCallsTime.toSec()/plannerCalls, plannerCalls);
    }
//
//    if (srv.response.error_code.val == moveit_msgs::ArmNavigationErrorCodes::SUCCESS)
//    {
//        ROS_INFO("%s Got a putdown pose.", logName.c_str());
//        return true;
//    }
//
//    ROS_WARN("%s GetPutdownPose failed. Reason: %s (%d)", logName.c_str(),
//            moveit_msgs::armNavigationErrorCodeToString(srv.response.error_code).c_str(),
//            srv.response.error_code.val);
    return true;
}

string writePoseToString(const geometry_msgs::Pose& pose)
{
    std::stringstream stream;
    stream.precision(5);
    stream << std::fixed;
    stream << pose.position.x << separator;
    stream << pose.position.y << separator;
    stream << pose.position.z << separator;
    stream << pose.orientation.x << separator;
    stream << pose.orientation.y << separator;
    stream << pose.orientation.z << separator;
    stream << pose.orientation.w << separator;
    return stream.str();
}

bool readPoseFromString(const string cacheValue, geometry_msgs::Pose& pose)
{
    std::stringstream stream;
    stream << cacheValue;
    vector<double> coordinates;
    coordinates.resize(7);
    for(size_t i = 0; ! stream.eof() && i < coordinates.size(); i++)
    {
        stream >> coordinates[i];
    }
    if (!stream.good())
        return false;
    pose.position.x = coordinates[0];
    pose.position.y = coordinates[1];
    pose.position.z = coordinates[2];
    pose.orientation.x = coordinates[3];
    pose.orientation.y = coordinates[4];
    pose.orientation.z = coordinates[5];
    pose.orientation.w = coordinates[6];
    return true;
}

string createCacheKey(const string& putdownObject,
        const string& arm,
        const string& staticObject,
        const map<string, geometry_msgs::Pose>& movableObjects,
        const map<string, string>& objectsOnStatic)
{
    ROS_DEBUG("%s createCacheKey %s %s %s", logName.c_str(), putdownObject.c_str(), arm.c_str(), staticObject.c_str());

    // using type for caching, doesn't matter which object it is.
    string putdownObjectType = putdownObject;
    int putdownObjectTypeId;
    // assuming that something like cup0 identifies a cup
    // we only put the type of object into the key as it doesn't matter
    // which one is there.
    splitNamedId(putdownObject, putdownObjectType, putdownObjectTypeId);

    std::stringstream stream;
    stream << std::fixed << putdownObjectType << arm << staticObject;
    for (map<string, string>::const_iterator objectIt = objectsOnStatic.begin(); objectIt != objectsOnStatic.end(); objectIt++)
    {
        ROS_DEBUG("%s object %s on %s", logName.c_str(), objectIt->first.c_str(), objectIt->second.c_str());
        if (objectIt->second == staticObject)
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

PutdownRequest createPutdownRequest(const string& putdownObject,
        const string& arm,
        const string& staticObject,
        const map<string, geometry_msgs::Pose>& movableObjects,
        const map<string, string>& objectsOnStatic)
{
    ROS_DEBUG("%s createPutdownRequest %s %s %s", logName.c_str(), putdownObject.c_str(), arm.c_str(), staticObject.c_str());
    PutdownRequest pr;
    pr.putdownObject = putdownObject;

    string pdoTypeName = putdownObject;
    int pdoTypeId;
    // assuming that something like cup0 identifies a cup
    // we only put the type of object into the key as it doesn't matter
    // which one is there.
    splitNamedId(putdownObject, pdoTypeName, pdoTypeId);
    pr.putdownObjectType = pdoTypeName;

    pr.arm = arm;
    pr.staticObject = staticObject;

    pr.computed = false;
    pr.success = false;

    // Compute the movable objects on staticObject
    for(map<string, string>::const_iterator objectIt = objectsOnStatic.begin(); objectIt != objectsOnStatic.end(); objectIt++) {
        ROS_DEBUG("CPR: %s object %s on %s", logName.c_str(), objectIt->first.c_str(), objectIt->second.c_str());
        if (objectIt->second == staticObject)
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

            PutdownRequest::MovableObject mo;
            mo.id = objectIt->first;
            mo.type = objectIdentifier;
            mo.pose = pose;
            pr.objectsOnStatic.push_back(mo);
        }
    }

    return pr;
}


bool findPutdownPose(const ParameterList & parameterList,
        predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback,
        geometry_msgs::Pose& putdown_pose)
{
    // (canPutdown ?o - movable_object ?a - arm ?s - static_object ?g - manipulation_location)
    ROS_ASSERT(parameterList.size() >= 4);
    Parameter putdown_object = parameterList[0];
    Parameter arm = parameterList[1];
    Parameter static_object = parameterList[2];
    Parameter robot_location = parameterList[3];

    // read state
    geometry_msgs::Pose robotPose;
    map<string, geometry_msgs::Pose> movableObjects;
    GraspedObjectMap graspedObjects;
    map<string, string> objectsOnStatic;
    set<string> openDoors;
    moveit_msgs::PlanningScene world = PlanningSceneInterface::instance()->getCurrentScene();
    if (! TidyupPlanningSceneUpdater::readState(robot_location.value, predicateCallback, numericalFluentCallback, robotPose, movableObjects, graspedObjects, objectsOnStatic, openDoors))
    {
        ROS_ERROR("%s read state failed.", logName.c_str());
        return false;
    }

    bool graspedOK = false;
    for(GraspedObjectMap::iterator it = graspedObjects.begin(); it!=graspedObjects.end();it++) {
        if(it->first == putdown_object.value && it->second.first == arm.value) {
            graspedOK = true;
        }
    }
    if(!graspedOK) {
        ROS_ERROR("findPutdownPose called for %s in %s, but it is not grasped.", putdown_object.value.c_str(), arm.value.c_str());
    }

    tidyup_msgs::GetPutdownPose srv;
    srv.request.static_object = static_object.value;
    srv.request.putdown_object = putdown_object.value;
    srv.request.arm = arm.value;
    ROS_INFO("%s putdown request: %s, %s, %s, %s", logName.c_str(), parameterList[0].value.c_str(), parameterList[1].value.c_str(), parameterList[2].value.c_str(), parameterList[3].value.c_str());

    string partialStateCacheKey = createCacheKey(srv.request.putdown_object, srv.request.arm, srv.request.static_object, movableObjects, objectsOnStatic);
    ROS_DEBUG("%s cacheKey: %s", logName.c_str(), partialStateCacheKey.c_str());
    PutdownRequest pdr = createPutdownRequest(srv.request.putdown_object, srv.request.arm, srv.request.static_object, movableObjects, objectsOnStatic);
    PutdownRequest pdrMatch;
    PutdownRequest pdrMatchGlobal;
    bool subsumptionMatch = g_SubsumptionCache->query(pdr, pdrMatch);
    bool subsumptionGlobalMatch = g_SubsumptionCacheGlobal->query(pdr, pdrMatchGlobal);
    if(g_EvaluatePutdownCaching) {
        if(subsumptionMatch) {
            g_PutdownCachingEvaluation.recordSubsumedCacheQueryLocal(partialStateCacheKey,
                    true, pdrMatch.computationTime);
        }
        if(subsumptionGlobalMatch) {
            g_PutdownCachingEvaluation.recordSubsumedCacheQueryGlobal(partialStateCacheKey,
                    true, pdrMatchGlobal.computationTime);  // starting of with a hit, so
                    // the time shouldn't matter. After this, we can't ever get a miss.
        }
    }
    if(subsumptionMatch && !subsumptionGlobalMatch) {
        ROS_ERROR("\n\n\nGot subsumption match, but not global match.\n\n\n");
        ROS_ERROR("Local Query:");
        g_SubsumptionCache->debugQuery(pdr);
        ROS_ERROR("Global Query:");
        g_SubsumptionCacheGlobal->debugQuery(pdr);
    }

    // retrieved cached result
    string cacheValue;
    bool partialWasLocal;
    if (paramCache.get(partialStateCacheKey, cacheValue, &partialWasLocal))
    {
        if(partialWasLocal && !subsumptionMatch) {
            ROS_ERROR("\n\n\nGot partial state match, but no subsumption match!\n\n\n");
            ROS_ERROR("Local Query:");
            g_SubsumptionCache->debugQuery(pdr);
        }
        if(!subsumptionGlobalMatch) {
            ROS_ERROR("\n\n\nGot partial state match, but no global subsumption match!\n\n\n");
            ROS_ERROR("Global Query:");
            g_SubsumptionCacheGlobal->debugQuery(pdr);
        }

        if(g_EvaluatePutdownCaching) {
            // partial key cache hit.
            double time;
            if(paramCache.getTime(partialStateCacheKey, time)) {
                // Check if full state would have hit and record accordingly.
                string fullStateCacheKey = computeFullStateCacheKey(parameterList, predicateCallback, numericalFluentCallback);
                string dummyValue;
                bool fullWasLocal;
                bool fullHit = paramCache.get(fullStateCacheKey, dummyValue, &fullWasLocal);

                g_PutdownCachingEvaluation.recordNoCacheQuery(partialStateCacheKey, time);
                g_PutdownCachingEvaluation.recordFullCacheQueryGlobal(partialStateCacheKey, fullHit, time);
                // partial hit branch
                g_PutdownCachingEvaluation.recordPartialCacheQueryGlobal(partialStateCacheKey, true, time);

                g_PutdownCachingEvaluation.recordFullCacheQueryLocal(partialStateCacheKey,
                        fullHit && fullWasLocal, time);
                g_PutdownCachingEvaluation.recordPartialCacheQueryLocal(partialStateCacheKey,
                        partialWasLocal, time);

                // this should only happen, if the partial hit was global
                if(!subsumptionMatch) {
                    g_PutdownCachingEvaluation.recordSubsumedCacheQueryLocal(partialStateCacheKey,
                            false, time);
                }
                // this should never happen
                if(!subsumptionGlobalMatch) {
                    g_PutdownCachingEvaluation.recordSubsumedCacheQueryGlobal(partialStateCacheKey,
                            false, time);
                }
                // Also set the full state key to be able to determine full state
                // hits and misses (globally).
                paramCache.set(fullStateCacheKey, cacheValue, true);
            }
        }

        if(!partialWasLocal && !subsumptionMatch) {
            // this wasn't a local hit and subsumption (local) doesn't have this yet.
            double time;
            if(paramCache.getTime(partialStateCacheKey, time)) {
                pdr.computed = true;
                pdr.computationTime =  time;
                pdr.success = (cacheValue != "impossible");
                readPoseFromString(cacheValue, pdr.putdownPose);
                g_SubsumptionCache->put(pdr);
            }
        }

        // cache hit. either 'impossible' or a valid pose
        if (cacheValue == "impossible")
        {
            ROS_DEBUG("%s cache hit: impossible", logName.c_str());
            if(subsumptionMatch) {
                if(pdrMatch.success) {
                    ROS_ERROR("Subsumption match fail: Reported SUCCESS for failed cached request.");
                    ROS_ERROR("Local Query:");
                    g_SubsumptionCache->debugQuery(pdr);
                }
            }
            if(subsumptionGlobalMatch) {
                if(pdrMatchGlobal.success) {
                    ROS_ERROR("Subsumption global match fail: Reported SUCCESS for failed cached request.");
                    ROS_ERROR("Global Query:");
                    g_SubsumptionCacheGlobal->debugQuery(pdr);
                }
            }
            return false;
        }
        else
        {
            if (! readPoseFromString(cacheValue, putdown_pose))
            {
                ROS_ERROR("%s could not read cached value. cache may be corrupt.", logName.c_str());
                return false;
            }
            if(subsumptionMatch) {
                if(!pdrMatch.success) {
                    ROS_ERROR("Subsumption match fail: Reported FAIL for successfull cached request.");
                    ROS_ERROR("Local Query:");
                    g_SubsumptionCache->debugQuery(pdr);
                }
                // poses might be different as subsumed is more constrained
            }
            if(subsumptionGlobalMatch) {
                if(!pdrMatchGlobal.success) {
                    ROS_ERROR("Subsumption global match fail: Reported FAIL for successfull cached request.");
                    ROS_ERROR("Global Query:");
                    g_SubsumptionCacheGlobal->debugQuery(pdr);
                }
                // poses might be different as subsumed is more constrained
            }
            ROS_DEBUG("%s cache hit: pose from cache", logName.c_str());
            return true;
        }
    } else if(subsumptionMatch || subsumptionGlobalMatch) {
        if(subsumptionMatch) {
            ROS_INFO("Got subsumption match, although partial state didn't match - good!");
            ROS_INFO("Local Query:");
            g_SubsumptionCache->debugQuery(pdr);
        }
        if(subsumptionGlobalMatch) {
            ROS_INFO("Got subsumption global match, although partial state didn't match - good!");
            ROS_INFO("Global Query:");
            g_SubsumptionCacheGlobal->debugQuery(pdr);
        }
    }

    ros::WallTime startCallTime = ros::WallTime::now();
    // no cache entry, set planning scene
    PlanningSceneInterface::instance()->resetPlanningScene();
    ROS_DEBUG("%s set planning scene", logName.c_str());
    // dont need to send the diff as its send with the request.
    if (! TidyupPlanningSceneUpdater::update(robotPose, movableObjects, graspedObjects, openDoors, false))
    {
        ROS_ERROR("%s update planning scene failed.", logName.c_str());
        return false;
    }

    // set planning scene in server call
    srv.request.planning_scene = PlanningSceneInterface::instance()->getCurrentScene();

    // call putdown service
    ROS_INFO("%s call putdown service", logName.c_str());
    if (! callFindPutdownPoseService(srv))
    {
        ROS_ERROR("%s service call failed", logName.c_str());
        return false;
    }
    ros::WallDuration callDuration = ros::WallTime::now() - startCallTime;

    if(g_EvaluatePutdownCaching) {
        // full miss
        double time = callDuration.toSec();
        // Check if full state would have hit and record accordingly.
        string fullStateCacheKey = computeFullStateCacheKey(parameterList, predicateCallback, numericalFluentCallback);
        g_PutdownCachingEvaluation.recordNoCacheQuery(partialStateCacheKey, time);
        g_PutdownCachingEvaluation.recordFullCacheQueryGlobal(partialStateCacheKey, false, time);
        g_PutdownCachingEvaluation.recordPartialCacheQueryGlobal(partialStateCacheKey, false, time);
        g_PutdownCachingEvaluation.recordFullCacheQueryLocal(partialStateCacheKey, false, time);
        g_PutdownCachingEvaluation.recordPartialCacheQueryLocal(partialStateCacheKey, false, time);
        paramCache.set(fullStateCacheKey, "NA", true);  // don't care about the actual value for full
        if(!subsumptionMatch) {
            g_PutdownCachingEvaluation.recordSubsumedCacheQueryLocal(partialStateCacheKey, false, time);
        }
        if(!subsumptionGlobalMatch) {
            g_PutdownCachingEvaluation.recordSubsumedCacheQueryGlobal(partialStateCacheKey, false, time);
        }
    }

    if (srv.response.error_code.val == srv.response.error_code.PLANNING_FAILED)
    {
        ROS_INFO("%s service returned: impossible", logName.c_str());
        paramCache.set(partialStateCacheKey, "impossible", true, callDuration.toSec());
        if(!subsumptionMatch) {
            pdr.computed = true;
            pdr.success = false;
            pdr.computationTime = callDuration.toSec();
            g_SubsumptionCache->put(pdr);
        } else {    // subsumptionMatch
            if(pdrMatch.success) {
                ROS_ERROR("Subsumption match fail: Reported SUCCESS for failed request.");
                ROS_ERROR("Local Query:");
                g_SubsumptionCache->debugQuery(pdr);
            }
        }
        if(!subsumptionGlobalMatch) {
            pdr.computed = true;
            pdr.success = false;
            pdr.computationTime = callDuration.toSec();
            g_SubsumptionCacheGlobal->put(pdr);
        } else {    // subsumptionMatch
            if(pdrMatchGlobal.success) {
                ROS_ERROR("Subsumption global match fail: Reported SUCCESS for failed request.");
                ROS_ERROR("Global Query:");
                g_SubsumptionCacheGlobal->debugQuery(pdr);
            }
        }
        //std::cout << "SET paramCache fail entry" << std::endl;
        //ROS_INFO("ModuleParamCache local");
        //paramCache.dump();
        //ROS_INFO("g_SubsumptionCache");
        //g_SubsumptionCache->dump();
        //ROS_INFO("g_SubsumptionCacheGlobal");
        //g_SubsumptionCacheGlobal->dump();

        return false;
    }
    else if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
    {
        // insert results into cache
        ROS_INFO("%s service returned: pose found, adding to cache", logName.c_str());
        putdown_pose = srv.response.putdown_pose.pose;
        paramCache.set(partialStateCacheKey, writePoseToString(putdown_pose), true, callDuration.toSec());
        if(!subsumptionMatch) {
            pdr.computed = true;
            pdr.success = true;
            pdr.computationTime = callDuration.toSec();
            pdr.putdownPose = putdown_pose;
            g_SubsumptionCache->put(pdr);
        } else {
            if(!pdrMatch.success) {
                ROS_ERROR("Subsumption match fail: Reported FAIL for successfull request.");
                ROS_ERROR("Local Query:");
                g_SubsumptionCache->debugQuery(pdr);
            } else {
                // Poses might be different in this case as the subsumed request might have been more constrained.
            }
        }
        if(!subsumptionGlobalMatch) {
            pdr.computed = true;
            pdr.success = true;
            pdr.computationTime = callDuration.toSec();
            pdr.putdownPose = putdown_pose;
            g_SubsumptionCacheGlobal->put(pdr);
        } else {
            if(!pdrMatchGlobal.success) {
                ROS_ERROR("Subsumption global match fail: Reported FAIL for successfull request.");
                ROS_ERROR("Global Query:");
                g_SubsumptionCacheGlobal->debugQuery(pdr);
            }
        }

        //std::cout << "SET paramCache success entry" << std::endl;
        //ROS_INFO("ModuleParamCache local");
        //paramCache.dump();
        //ROS_INFO("g_SubsumptionCache");
        //g_SubsumptionCache->dump();
        //ROS_INFO("g_SubsumptionCacheGlobal");
        //g_SubsumptionCacheGlobal->dump();
        return true;
    }

    ROS_ERROR("%s GetPutdownPose failed. Reason: %d", logName.c_str(),
            srv.response.error_code.val);
    return false;
}


double canPutdown(const ParameterList & parameterList,
        predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback,
        int relaxed)
{
    if (relaxed != 0)
    {
        // skip computation for heuristic
        return 0.0;
    }
    if (g_Debug)
    { // prevent spamming ROS_DEBUG calls unless we really want debug
        // debugging raw planner calls
        static unsigned int calls = 0;
        calls++;
        if (calls % 10000 == 0)
        {
            ROS_DEBUG("%s Got %d putdown module calls.\n", logName.c_str(), calls);
        }
    }

    geometry_msgs::Pose putdownPose;
    if (! findPutdownPose(parameterList, predicateCallback, numericalFluentCallback, putdownPose))
    {
        return INFINITE_COST;
    }
    return 0.0;
}

double wipePointFree(const ParameterList & parameterList, predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    // (wipePointFree ?w - wipe_point ?l - manipulation_location)
    ROS_ASSERT(parameterList.size() >= 3);
    Parameter wipe_point = parameterList[0];
    Parameter robot_location = parameterList[1];
    Parameter static_object = parameterList[2];

    // read state
    geometry_msgs::Pose robotPose;
    map<string, geometry_msgs::Pose> movableObjects;
    GraspedObjectMap graspedObjects;
    map<string, string> objectsOnStatic;
    set<string> openDoors;
    moveit_msgs::PlanningScene world = PlanningSceneInterface::instance()->getCurrentScene();
    if (! TidyupPlanningSceneUpdater::readState(robot_location.value, predicateCallback, numericalFluentCallback, robotPose, movableObjects, graspedObjects, objectsOnStatic, openDoors))
    {
        ROS_ERROR("%s read state failed.", logName.c_str());
        return INFINITE_COST;
    }
    // small HACK: pass the name of the wipe point as arm, to get unique cache keys for each point
    string partialStateCacheKey = createCacheKey(wipe_point.value, wipe_point.value, static_object.value, movableObjects, objectsOnStatic);
    ROS_DEBUG("%s cacheKey: %s", logName.c_str(), partialStateCacheKey.c_str());
    // retrieved cached result
    string cacheValue;
    if (paramCache.get(partialStateCacheKey, cacheValue))
    {
        // cache hit. either 'impossible' or 'possible'
        if (cacheValue == "impossible")
        {
            ROS_DEBUG("%s cache hit: impossible", logName.c_str());
            return INFINITE_COST;
        }
        else
        {
            ROS_DEBUG("%s cache hit: possible", logName.c_str());
            return 0.0;
        }
    }

    ros::WallTime startCallTime = ros::WallTime::now();
    geometry_msgs::Point wipePoint;
    if (! TidyupPlanningSceneUpdater::fillPointFromState(wipePoint, wipe_point.value, numericalFluentCallback))
    {
        ROS_ERROR("%s read wipe point %s failed.", logName.c_str(), wipe_point.value.c_str());
        return INFINITE_COST;
    }
    // check diatances to objects
    ParameterList params;
    params.push_back(Parameter("", "", "object"));
    params.push_back(static_object);
    for (map<string, geometry_msgs::Pose>::const_iterator objectIterator = movableObjects.begin();
            objectIterator != movableObjects.end(); objectIterator++)
    {
        map<string, string>::const_iterator onStaticIt = objectsOnStatic.find(objectIterator->first);
        if (onStaticIt != objectsOnStatic.end())
        {
            if (onStaticIt->second == static_object.value)
            {
                // on the table we are interested in, compute distance
                geometry_msgs::Pose object_pose = objectIterator->second;
                double distance = hypot(wipePoint.x - object_pose.position.x, wipePoint.y - object_pose.position.y);
                if (distance < 0.17)
                {
                    ROS_INFO ("%s wipePointFree: object %s is to close to wipe point %s", logName.c_str(), objectIterator->first.c_str(), wipe_point.value.c_str());
                    // insert into chache
                    ros::WallTime endCallTime = ros::WallTime::now();
                    paramCache.set(partialStateCacheKey, "impossible", true, (endCallTime - startCallTime).toSec());
                    return INFINITE_COST;
                }
            }
        }
    }
    // insert into chache
    ros::WallTime endCallTime = ros::WallTime::now();
    paramCache.set(partialStateCacheKey, "possible", true, (endCallTime - startCallTime).toSec());
    return 0;
}

int updatePutdownPose(const ParameterList & parameterList, predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback, int relaxed,
        std::vector<double> & writtenVars)
{
    if(relaxed != 0) {
        ROS_ERROR("%s: called with relaxed %d, can't handle this.", __func__, relaxed);
        return 0;
    }

    geometry_msgs::Pose putdownPose;
    if (! findPutdownPose(parameterList, predicateCallback, numericalFluentCallback, putdownPose))
    {
        return 0;
    }
    // write the pose to state:
    writtenVars[0] = putdownPose.position.x;
    writtenVars[1] = putdownPose.position.y;
    writtenVars[2] = putdownPose.position.z;
    writtenVars[3] = putdownPose.orientation.x;
    writtenVars[4] = putdownPose.orientation.y;
    writtenVars[5] = putdownPose.orientation.z;
    writtenVars[6] = putdownPose.orientation.w;
    return 1;
}

