#ifndef ACTION_EXECUTOR_INSPECT_OBJECT_H
#define ACTION_EXECUTOR_INSPECT_OBJECT_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"

namespace object_manipulation_actions
{

    class ActionExecutorInspectObject : public continual_planning_executive::ActionExecutorInterface
    {
        public:

    	ActionExecutorInspectObject();
		~ActionExecutorInspectObject();

		virtual void initialize(const std::deque<std::string> & arguments);

		virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

		virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

		virtual void cancelAction();

        private:

		std::string action_name_;
		std::string predicate_object_inspected_;

		bool updateSymbolicState(SymbolicState& currentState, const std::string& object);

    };

};

#endif

