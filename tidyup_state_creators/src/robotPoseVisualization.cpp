#include "tidyup_state_creators/robotPoseVisualization.h"
#include <moveit_msgs/RobotState.h>
//#include "state_transformer/GetRobotMarker.h"
//#include "arm_navigation_msgs/GetRobotState.h"

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>



RobotPoseVisualization::RobotPoseVisualization()
{
}

RobotPoseVisualization::~RobotPoseVisualization()
{
}

bool RobotPoseVisualization::initialize()
{
    bool ret = true;
    ros::NodeHandle nh;

//    // initialize initState from environemtn server
//    ros::ServiceClient srvGetRobotState =
//        nh.serviceClient<arm_navigation_msgs::GetRobotState>("/environment_server/get_robot_state");
//    if(!srvGetRobotState.exists()) {
//        ROS_FATAL("Service /environment_server/get_robot_state does not seem to exist.");
//        ret = false;
//    } else {
//        arm_navigation_msgs::GetRobotState srv;
//        if(!srvGetRobotState.call(srv)) {
//            ROS_FATAL("Call to /environment_server/get_robot_state failed.");
//            ret = false;
//        } else {
//            _initState = srv.response.robot_state;
//            _currentState = _initState;
//        }
//    }
//
//    // setup service client for get robot marker
//    _srvGetRobotMarker = nh.serviceClient<state_transformer::GetRobotMarker>("/get_robot_marker");
//    if(!_srvGetRobotMarker.exists()) {
//        ROS_FATAL("Service /get_robot_marker does not seem to exist");
//        ret = false;
//    }


/*
	ros::ServiceClient getPlanningSceneClient =
			nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene", true);
	if (!getPlanningSceneClient.exists()) {
		ROS_FATAL("Service /get_planning_scene does not seem to exist.");
		ret = false;
	} else {
		moveit_msgs::GetPlanningScene srv;
		srv.request.components.ROBOT_STATE;
		if (!getPlanningSceneClient.call(srv)) {
			ROS_FATAL("Call for ROBOT_STATE using service /get_planning_scene failed.");
			ret = false;
		} else {
			_initState = srv.response.scene.robot_state;
			_currentState = _initState;
		}
	}


	ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>( "tutorial_robot_state", 1 );

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_state::RobotState robot_state(robot_model_loader.getModel());
*/
    // Alternative: visit: https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/state_display/src/state_display_tutorial.cpp










    return ret;
}

void RobotPoseVisualization::resetRobotState()
{
    _currentState = _initState;
}

void RobotPoseVisualization::updateRobotStateJoints(const sensor_msgs::JointState & js)
{
    if(!currentStateInitialized()) {
        ROS_ERROR("%s: currentState not initialized", __func__);
        return;
    }

//    sensor_msgs::JointState & curJs = _currentState.joint_state;
//    for(unsigned int i = 0; i < js.name.size(); i++) {
//        // find matching joint name in _currentState.joint_state
//        for(unsigned int j = 0; j < curJs.name.size(); j++) {
//            if(js.name[i] == curJs.name[j]) {
//                curJs.position[j] = js.position[i];
//            }
//        }
//    }
}

void RobotPoseVisualization::updateRobotStatePose(const geometry_msgs::PoseStamped & pose)
{
    if(!currentStateInitialized()) {
        ROS_ERROR("%s: currentState not initialized", __func__);
        return;
    }

//    _currentState.multi_dof_joint_state.frame_ids[0] = pose.header.frame_id;
//    _currentState.multi_dof_joint_state.poses[0] = pose.pose;
//
//    // super nasty hack: I thought the first multi_dof_joint_state being the pose of /base_footprint in /map
//    // actually gives the robot position.
//    // This seems to be ignored, but the joints floating_trans_x/y/z and floating_rot_x/y/z/w instead give the pose
//    sensor_msgs::JointState & js = _currentState.joint_state;
//    for(unsigned int i = 0; i < js.name.size(); i++) {
//        if(js.name[i] == "floating_trans_x")
//            js.position[i] = pose.pose.position.x;
//        if(js.name[i] == "floating_trans_y")
//            js.position[i] = pose.pose.position.y;
//        if(js.name[i] == "floating_trans_z")
//            js.position[i] = pose.pose.position.z;
//        if(js.name[i] == "floating_rot_x")
//            js.position[i] = pose.pose.orientation.x;
//        if(js.name[i] == "floating_rot_y")
//            js.position[i] = pose.pose.orientation.y;
//        if(js.name[i] == "floating_rot_z")
//            js.position[i] = pose.pose.orientation.z;
//        if(js.name[i] == "floating_rot_w")
//            js.position[i] = pose.pose.orientation.w;
//    }
}

visualization_msgs::MarkerArray RobotPoseVisualization::getMarkers(
        const std_msgs::ColorRGBA & color, const std::string & ns)
{
//    if(!currentStateInitialized()) {
//        ROS_ERROR("%s: currentState not initialized", __func__);
//        return visualization_msgs::MarkerArray();
//    }
//
//    state_transformer::GetRobotMarker srv;
//    srv.request.robot_state = _currentState;
//    srv.request.color = color;
//    srv.request.ns = ns;
//    srv.request.scale = 1.0;
//
//    if(!_srvGetRobotMarker.call(srv)) {
//        ROS_ERROR("Calling GetRobotMarker service failed.");
//    }
//    return srv.response.marker_array;
	visualization_msgs::MarkerArray a;
	return a;
}

bool RobotPoseVisualization::currentStateInitialized() const
{
//    return(!_currentState.joint_state.name.empty() && !_currentState.multi_dof_joint_state.poses.empty());
	return true;
}

