#include "object_manipulation_actions/actionExecutorDetectObjects.h"
#include <pluginlib/class_list_macros.h>
#include <tidyup_msgs/RequestObjectsGraspability.h>
#include "tidyup_utils/planning_scene_interface.h"
#include <set>

//PLUGINLIB_DECLARE_CLASS(object_manipulation_actions, action_executor_detect_objects,
//		object_manipulation_actions::ActionExecutorDetectObjects,
//		continual_planning_executive::ActionExecutorInterface)
PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorDetectObjects, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{
void ActionExecutorDetectObjects::initialize(const std::deque<std::string> & arguments)
{
    ActionExecutorService<tidyup_msgs::DetectGraspableObjects>::initialize(arguments);
    requestGraspability = false;
//        _nh->param("head_pointing_frame", head_pointing_frame, string("/openni_rgb_optical_frame"));
//        headControl = new actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction>(*_nh, "/head_traj_controller/point_head_action");
    string graspabilityServiceName = "/learned_grasping/request_objects_graspability";
    tidyLocationName = "table1";

    if (arguments.size() >= 3)
    {
        if (arguments[2] == "NULL")
        {
            requestGraspability = false;
        }
        else
        {
            graspabilityServiceName = arguments[2];
        }
    }
    if (arguments.size() >= 4)
    {
        tidyLocationName = arguments[3];
    }

    ROS_ASSERT(_nh);
    if (requestGraspability)
    {
        serviceClientGraspability = _nh->serviceClient<tidyup_msgs::RequestObjectsGraspability>(graspabilityServiceName);
        if (!serviceClientGraspability)
        {
            ROS_FATAL("Could not initialize service for RequestObjectsGraspability.");
        }
    }
}

//    bool ActionExecutorDetectObjects::executeBlocking(const DurativeAction & a, SymbolicState & current)
//    {
//        ROS_ASSERT(a.parameters.size() == 2);
//        string static_object = a.parameters[1];
//        pr2_controllers_msgs::PointHeadGoal head_control_msg;
//        PlanningSceneInterface* psi = PlanningSceneInterface::instance();
//        const arm_navigation_msgs::CollisionObject* object = psi->getCollisionObject(static_object);
//        if (object)
//        {
//            head_control_msg.target.point = object->poses[0].position;
//            head_control_msg.target.point.z -= 0.2;
//            head_control_msg.target.header.frame_id = object->header.frame_id;
//        }
//        else
//        {
//            head_control_msg.target.point.x = 0.8;
//            head_control_msg.target.point.y = 0.0;
//            head_control_msg.target.point.z = 0.7;
//            head_control_msg.target.header.frame_id = "/base_link";
//        }
//        head_control_msg.max_velocity = 0.25;
//        head_control_msg.min_duration = ros::Duration(0.5);
//        head_control_msg.pointing_frame = head_pointing_frame;
//        ROS_INFO("aligning head...");
//        headControl->sendGoalAndWait(head_control_msg);
//        ROS_INFO("head aligned");
//        bool success = ActionExecutorService<tidyup_msgs::DetectGraspableObjects>::executeBlocking(a, current);
//        return success;
//    }

bool ActionExecutorDetectObjects::fillGoal(tidyup_msgs::DetectGraspableObjects::Request & goal, const DurativeAction & a, const SymbolicState & current)
{
    if (!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
        ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);

    ROS_ASSERT(a.parameters.size() == 1);
    goal.static_object = findStaticObjectForLocation(a.parameters[0], current);
    return true;
}

void ActionExecutorDetectObjects::updateState(bool& success, tidyup_msgs::DetectGraspableObjects::Response & response, const DurativeAction & a, SymbolicState & current)
{
    ROS_INFO("DetectObjects returned result");
    if (success)
    {
        std::set<string> objectsOnStatic;
        ROS_INFO("DetectObjects succeeded.");
        ROS_ASSERT(a.parameters.size() == 1);
        std::string location = a.parameters[0];
        current.setBooleanPredicate("searched", location, true);
        current.setBooleanPredicate("recent-detected-objects", location, true);

        std::vector<tidyup_msgs::GraspableObject>& objects = response.objects;

        // find correct static object and set the "on" predicate
        string static_object = findStaticObjectForLocation(location, current);
        ROS_ASSERT(static_object != "");

        // remove objects form state, which were previously detected from this location
        Predicate p;
        p.name = "on";
        p.parameters.push_back("object");
        p.parameters.push_back(static_object);
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> objectRange = current.getTypedObjects().equal_range("movable_object");
        for (SymbolicState::TypedObjectConstIterator objectIterator = objectRange.first; objectIterator != objectRange.second; objectIterator++)
        {
            string object = objectIterator->second;
            p.parameters[0] = object;
            bool onStatic = false;
            if (current.hasBooleanPredicate(p, &onStatic))
            {
                if (onStatic)
                {
                    objectsOnStatic.insert(object);
                }
            }
        }

        for (std::vector<tidyup_msgs::GraspableObject>::iterator it = objects.begin(); it != objects.end(); it++)
        {
            tidyup_msgs::GraspableObject & object = *it;
            std::set<string>::iterator found = objectsOnStatic.find(object.name);
            if (found != objectsOnStatic.end())
            {
                objectsOnStatic.erase(found);
            }
            current.addObject(object.name, "movable_object");
            if (object.pose.header.frame_id.empty())
            {
                ROS_ERROR("DetectGraspableObjects returned empty frame_id for object: %s", object.name.c_str());
                object.pose.header.frame_id = "INVALID_FRAME_ID";
            }
            current.addObject(object.pose.header.frame_id, "frameid");
            current.addObject(object.name, "pose");
            current.setObjectFluent("frame-id", object.name, object.pose.header.frame_id);
            current.setNumericalFluent("x", object.name, object.pose.pose.position.x);
            current.setNumericalFluent("y", object.name, object.pose.pose.position.y);
            current.setNumericalFluent("z", object.name, object.pose.pose.position.z);
            current.setNumericalFluent("qx", object.name, object.pose.pose.orientation.x);
            current.setNumericalFluent("qy", object.name, object.pose.pose.orientation.y);
            current.setNumericalFluent("qz", object.name, object.pose.pose.orientation.z);
            current.setNumericalFluent("qw", object.name, object.pose.pose.orientation.w);
            current.setNumericalFluent("timestamp", object.name, object.pose.header.stamp.toSec());
            current.setBooleanPredicate("on", object.name + " " + static_object, true);
            //current.setObjectFluent("object-detected-from", object.name, location);
            // tidy-location: (tidy-location ?o ?s)
            current.setBooleanPredicate("tidy-location", object.name + " " + tidyLocationName, true);

            // add graspable predicates from current location
            //current.setBooleanPredicate("graspable-from", object.name + " " + location + " left_arm", object.reachable_left_arm);
            //current.setBooleanPredicate("graspable-from", object.name + " " + location + " right_arm", object.reachable_right_arm);
        }

        forEach(string unseen, objectsOnStatic)
        {
            current.removeObject(unseen, true);
        }

        if (!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
            ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);
    }
}

std::string ActionExecutorDetectObjects::findStaticObjectForLocation(const std::string& location, const SymbolicState & current) const
{
    Predicate p;
    string static_object;
    p.name = "static-object-at-location";
    p.parameters.push_back("object");
    p.parameters.push_back(location);
    pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> objectRange = current.getTypedObjects().equal_range("static_object");
    for (SymbolicState::TypedObjectConstIterator objectIterator = objectRange.first; objectIterator != objectRange.second; objectIterator++)
    {
        string object = objectIterator->second;
        p.parameters[0] = object;
        bool value = false;
        if (current.hasBooleanPredicate(p, &value))
        {
            if (value)
            {
                static_object = object;
                break;
            }
        }
    }
    return static_object;
}

}
;

