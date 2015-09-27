#ifndef ACTION_EXECUTOR_GRASP_OBJECT_GROUNDING_H
#define ACTION_EXECUTOR_GRASP_OBJECT_GROUNDING_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <symbolic_planning_utils/planning_scene_interface.h>
#include <grasp_provider_msgs/GenerateGraspsAction.h>


#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


namespace object_manipulation_actions
{
    class ActionExecutorPickupObjectGrounding : public continual_planning_executive::ActionExecutorInterface
    {
        public:

    	ActionExecutorPickupObjectGrounding();
		~ActionExecutorPickupObjectGrounding();

		virtual void initialize(const std::deque<std::string> & arguments);

		virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

		virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

		virtual void cancelAction();


        private:

		std::string action_name_;
		std::string predicate_inspected_recently_;
		ros::ServiceClient getPlanningSceneClient_;
		actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction> actionGenerateGrasps_;

		boost::shared_ptr<symbolic_planning_utils::PlanningSceneInterface> psi_;
    };

};


#endif
