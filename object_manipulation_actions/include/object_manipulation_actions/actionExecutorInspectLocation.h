#ifndef ACTION_EXECUTOR_INSPECT_LOCATION_H
#define ACTION_EXECUTOR_INSPECT_LOCATION_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/SingleJointPositionAction.h>
#include <control_msgs/SingleJointPositionFeedback.h>
#include <ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h>

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

        bool add_tables_;
        bool verify_planning_scene_update_;
        std::set<std::string> expected_objects_;

        double vDistHeadToTable_;
        ros::Duration actionTimeOut_;
        actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction> actionLiftTorso_;
        actionlib::SimpleActionClient<control_msgs::PointHeadAction> actionPointHead_;
        actionlib::SimpleActionClient<ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkAction> actionOrkToPs_;

        bool executeLiftTorso(const geometry_msgs::PoseStamped tablePose);
        void feedbackLiftTorso(const control_msgs::SingleJointPositionFeedback& feedback);
        bool executePointHead(const geometry_msgs::PoseStamped tablePose);
        bool executeUpdatePlanningSceneFromORK(SymbolicState& currentState,
        		const std::string tableName, const std::string manipulationLocation);

    };

};

#endif

