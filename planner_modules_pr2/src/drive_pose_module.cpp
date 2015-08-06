#include "planner_modules_pr2/drive_pose_module.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

//#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

//VERIFY_CONDITIONCHECKER_DEF(liftTorsoCost);
//VERIFY_CONDITIONCHECKER_DEF(needToLiftTorso);
//VERIFY_CONDITIONCHECKER_DEF(torsoLifted);
//VERIFY_APPLYEFFECT_DEF(updateTorsoPosition);

VERIFY_GROUNDINGMODULE_DEF(determineDrivePose);

// ________________________________________________________________________________________________
void drivePoseInit(int argc, char** argv)
{
	ROS_ASSERT(argc == 2);

    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);
    world_frame_ = tf::resolve(tfPrefix, argv[1]);


   	ROS_INFO_STREAM("drive_pose_modules::" << __func__ << ": param namespace: " << nhPriv.getNamespace() << "\n"
		"world frame: " << world_frame_);

    ROS_INFO("drive_pose_modules::%s: Initialized drive pose Module.", __func__);
}

// ________________________________________________________________________________________________
std::string determineDrivePose(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback, modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed, const void* statePtr)
{
    // 1. Get state and request (even if not needed - filter later)
    ROS_ASSERT(parameterList.size() == 2);
    std::string baseSurface = parameterList.at(1).value;

    std::string from, to;
    from = parameterList.at(0).value;
    to = parameterList.at(1).value;

    ROS_WARN("drive_pose_modules::%s: from: %s to: %s", __func__, from.c_str(), to.c_str());
//    ROS_WARN("drive_pose_modules::%s: baseSurface: %s", __func__, baseSurface.c_str());

    return baseSurface;
}

