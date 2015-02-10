#include "tidyup_state_creators/goalCreatorTidyupInitialize.h"
#include "tidyup_utils/geometryPoses.h"
#include "tidyup_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>

//PLUGINLIB_DECLARE_CLASS(tidyup_state_creators, goal_creator_tidyup_initialize,
//        tidyup_state_creators::GoalCreatorTidyupInitialize, continual_planning_executive::GoalCreator)
PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::GoalCreatorTidyupInitialize, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{
    GoalCreatorTidyupInitialize::GoalCreatorTidyupInitialize()
    {
    }

    GoalCreatorTidyupInitialize::~GoalCreatorTidyupInitialize()
    {
    }

    void GoalCreatorTidyupInitialize::initialize(const std::deque<std::string> & arguments)
    {

    }

    bool GoalCreatorTidyupInitialize::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        ros::NodeHandle nhPriv("~");
        // first add the type hierarchy
        currentState.addSuperType("pose", "pose");
        currentState.addSuperType("frameid", "frameid");
        currentState.addSuperType("location", "pose");
        currentState.addSuperType("manipulation_location", "location");
        currentState.addSuperType("table", "pose");
        currentState.addSuperType("movable_object", "pose");
        currentState.addSuperType("arm", "arm");
        currentState.addSuperType("arm_state", "arm_state");
//        currentState.addSuperType("door_location", "location");
//        currentState.addSuperType("door_in_location", "door_location");
//        currentState.addSuperType("door_out_location", "door_location");
//        currentState.addSuperType("room", "room");
//        currentState.addSuperType("static_object", "static_object");
//        currentState.addSuperType("door", "door");
        goal.addSuperType("pose", "pose");
        goal.addSuperType("frameid", "frameid");
        goal.addSuperType("location", "pose");
        goal.addSuperType("manipulation_location", "location");
        goal.addSuperType("table", "pose");
        goal.addSuperType("movable_object", "pose");
        goal.addSuperType("arm", "arm");
        goal.addSuperType("arm_state", "arm_state");
//        goal.addSuperType("door_location", "location");
//        goal.addSuperType("door_in_location", "door_location");
//        goal.addSuperType("door_out_location", "door_location");
//        goal.addSuperType("room", "room");
//        goal.addSuperType("static_object", "static_object");
//        goal.addSuperType("door", "door");

        currentState.printSuperTypes();

        // load object_locations
        std::string locationsFile;
        // load grasp_locations
        if(!nhPriv.getParam("locations", locationsFile)) {
            ROS_ERROR("Could not get ~locations parameter.");
            return false;
        }
        ROS_INFO("%s: file_name: %s", __PRETTY_FUNCTION__, locationsFile.c_str());
        // GeometryPoses contains std::map<std::string, geometry_msgs::PoseStamped> poses
        GeometryPoses locations = GeometryPoses();
        if(!locations.load(locationsFile)) {
            ROS_ERROR("Could not load locations from \"%s\".", locationsFile.c_str());
            return false;
        }

        // TODO: why these sets? They are filled but never used.
        std::set<string> rooms;
        std::set<string> doors;
        std::set<string> static_objects;
        forEach(const GeometryPoses::NamedPose & np, locations.getPoses())
        {
            const string& location = np.first;
            currentState.setNumericalFluent("timestamp", location, np.second.header.stamp.toSec());
            currentState.addObject(np.second.header.frame_id, "frameid");
            currentState.setObjectFluent("frame-id", location, np.second.header.frame_id);
            currentState.setNumericalFluent("x", location, np.second.pose.position.x);
            currentState.setNumericalFluent("y", location, np.second.pose.position.y);
            currentState.setNumericalFluent("z", location, np.second.pose.position.z);
            currentState.setNumericalFluent("qx", location, np.second.pose.orientation.x);
            currentState.setNumericalFluent("qy", location, np.second.pose.orientation.y);
            currentState.setNumericalFluent("qz", location, np.second.pose.orientation.z);
            currentState.setNumericalFluent("qw", location, np.second.pose.orientation.w);

            // When using different rooms
            // additional fluents
            // location name scheme: <type>typeName_AdditionalName_<room>roomName
            const vector<string>& nameParts = StringUtil::split(location, "_");
            const string& room = nameParts[nameParts.size()-1];
            const string& type = nameParts[0];
            string name = nameParts[0];
//            for(size_t i = 1; i < nameParts.size()-1; i++)
//                name += "_" + nameParts[i];
// commented out by me - resulting in KeyError = 'room'
//            rooms.insert(room);
//            currentState.addObject(room, "room");
//            currentState.setObjectFluent("location-in-room", location, room);
//
//            if (StringUtil::startsWith(type, "door"))
//            {
//                doors.insert(name);
//                currentState.addObject(name, "door");
//
//                // door_in_location
//                currentState.setObjectFluent("belongs-to-door", location, name);
//                currentState.addObject(location, "door_in_location");
//                goal.addObject(location, "door_in_location");
//
//                // also create the matching door_out_location as rotZ by 180 deg
//                geometry_msgs::PoseStamped outPose = np.second; // copy everything, only switch orientation
//                tf::Quaternion rot180 = tf::createQuaternionFromYaw(M_PI);
//                tf::Quaternion locRot;
//                tf::quaternionMsgToTF(outPose.pose.orientation, locRot);
//                tf::quaternionTFToMsg(locRot * rot180, outPose.pose.orientation);
//
//                string outLocation = location + "_out";
//                currentState.setObjectFluent("belongs-to-door", outLocation, name);
//                currentState.addObject(outLocation, "door_out_location");
//                goal.addObject(outLocation, "door_out_location");
//
//                currentState.setNumericalFluent("timestamp", outLocation, outPose.header.stamp.toSec());
//                currentState.setObjectFluent("frame-id", outLocation, outPose.header.frame_id);
//                currentState.setNumericalFluent("x", outLocation, outPose.pose.position.x);
//                currentState.setNumericalFluent("y", outLocation, outPose.pose.position.y);
//                currentState.setNumericalFluent("z", outLocation, outPose.pose.position.z);
//                currentState.setNumericalFluent("qx", outLocation, outPose.pose.orientation.x);
//                currentState.setNumericalFluent("qy", outLocation, outPose.pose.orientation.y);
//                currentState.setNumericalFluent("qz", outLocation, outPose.pose.orientation.z);
//                currentState.setNumericalFluent("qw", outLocation, outPose.pose.orientation.w);
//                //currentState.setObjectFluent("location-in-room", outLocation, room);
//                currentState.setObjectFluent("location", outLocation, room);
//            }
//            else
            {
                static_objects.insert(name);
                //currentState.addObject(name, "static_object");
                currentState.addObject(name, "table");
                //currentState.setBooleanPredicate("static-object-at-location", name + " " + location, true);
                // currentState.setBooleanPredicate("location-near-table", name + " " + location, true);
                currentState.setBooleanPredicate("location-near-table", location + " " + name, true);
                currentState.addObject(location, "manipulation_location");
                goal.addObject(location, "manipulation_location");
            }
        }

//		  // Not defined in domain predicates
//        currentState.setBooleanPredicate("can-grasp", "right_arm", true);
//        currentState.setBooleanPredicate("can-grasp", "left_arm", true);

        currentState.setObjectFluent("arm-state", "right_arm", "arm_unknown");
        currentState.setObjectFluent("arm-state", "left_arm", "arm_unknown");

        return true;
    }

};

