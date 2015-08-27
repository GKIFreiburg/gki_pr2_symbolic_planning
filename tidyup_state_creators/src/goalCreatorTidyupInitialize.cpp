#include "tidyup_state_creators/goalCreatorTidyupInitialize.h"
#include "tidyup_utils/geometryPoses.h"
#include "tidyup_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>
#include <moveit_msgs/LoadMap.h>

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

        goal.addSuperType("pose", "pose");
        goal.addSuperType("frameid", "frameid");
        goal.addSuperType("location", "pose");
        goal.addSuperType("manipulation_location", "location");
        goal.addSuperType("table", "pose");
        goal.addSuperType("movable_object", "pose");
        goal.addSuperType("arm", "arm");
        goal.addSuperType("arm_state", "arm_state");

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
            ROS_ERROR("GoalCreatorTidyupInitialize::%s: Could not load locations from \"%s\".", __func__, locationsFile.c_str());
            return false;
        }

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

			currentState.addObject(name, "table");
			currentState.setBooleanPredicate("location-near-table", location + " " + name, true);
			currentState.addObject(location, "manipulation_location");
			goal.addObject(location, "manipulation_location");
        }

        currentState.setObjectFluent("arm-state", "right_arm", "arm_unknown");
        currentState.setObjectFluent("arm-state", "left_arm", "arm_unknown");

        return true;
    }

};

