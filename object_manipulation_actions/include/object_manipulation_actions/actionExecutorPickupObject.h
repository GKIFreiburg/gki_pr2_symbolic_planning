#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"

#include <actionlib/client/simple_action_client.h>
#include <grasp_provider_msgs/GenerateGraspsAction.h>

namespace object_manipulation_actions
{
    class ActionExecutorPickupObject : public continual_planning_executive::ActionExecutorInterface
    {
        public:

    	ActionExecutorPickupObject();
		~ActionExecutorPickupObject();

		virtual void initialize(const std::deque<std::string> & arguments);

		virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

		virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

		virtual void cancelAction();


        private:

		std::string action_name_;
		ros::ServiceClient getPlanningSceneClient_;
		actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction> actionGenerateGrasps_;
    };

};


#endif
