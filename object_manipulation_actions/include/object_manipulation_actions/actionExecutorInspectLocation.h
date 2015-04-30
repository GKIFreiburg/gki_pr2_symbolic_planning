#ifndef ACTION_EXECUTOR_INSPECT_LOCATION_H
#define ACTION_EXECUTOR_INSPECT_LOCATION_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/SingleJointPositionAction.h>
#include <ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h>
#include <symbolic_planning_utils/planning_scene_interface.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

namespace object_manipulation_actions
{
    class ActionExecutorInspectLocation : public continual_planning_executive::ActionExecutorInterface
    {
        public:

    	ActionExecutorInspectLocation();
		~ActionExecutorInspectLocation();

		virtual void initialize(const std::deque<std::string> & arguments);

		virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

		virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

		virtual void cancelAction();


        private:
        // ROS Interface
        tf::TransformListener tf_;

        std::string action_topic_;
        std::string action_name_;
        std::vector<std::string> predicate_names_;

        // help variables for action liftTorso
        double torsoPosition_;
        bool setTorsoPosition_;
        int stallThreshold_;
        double startStallTime_; // in sec

        ros::Duration actionTimeOut_;
        actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction> actionLiftTorso_;
        actionlib::SimpleActionClient<control_msgs::PointHeadAction> actionPointHead_;
        actionlib::SimpleActionClient<ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkAction> actionOrkToPs_;
        boost::shared_ptr<symbolic_planning_utils::PlanningSceneInterface> psi_;

        // Parameters
        // UpdatePlanningSceneFromOrk-Parameters
        bool add_tables_;
        bool verify_planning_scene_update_;
        std::set<std::string> expected_objects_;
        // LiftTorso-Parameters
        double vdist_head_to_table_;
        double vdist_threshold_;
        double min_torso_vel_;

        // Action for lifting torso
        bool executeLiftTorso(const geometry_msgs::PoseStamped tablePose);
        // Checking if torso is stalled by reaching joint limits
        void feedbackLiftTorso(const control_msgs::SingleJointPositionFeedbackConstPtr& feedback);
        // Get position of torso, by sending an action request and reading position from feedback, then break
        void setTorsoPosition();
        // Action for pointing head
        bool executePointHead(const geometry_msgs::PoseStamped tablePose);
        // Action for object dectection
        bool executeUpdatePlanningSceneFromORK(SymbolicState& currentState,
        		const std::string& tableName, const std::string& manipulationLocation);

        void renameTableCollisionObject(const std::string& tableName);

    };

};

#endif

