#ifndef STATE_CREATOR_FROM_PLANNING_SCENE_H
#define STATE_CREATOR_FROM_PLANNING_SCENE_H

#include "continual_planning_executive/stateCreator.h"
#include <moveit_msgs/PlanningScene.h>
#include <set>

namespace tidyup_state_creators
{

	class StateCreatorFromPlanningScene: public continual_planning_executive::StateCreator
	{
		public:

			StateCreatorFromPlanningScene();
			~StateCreatorFromPlanningScene();

            /// Initialize the action from a list of arguments - should be called after creating the interface.
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

		private:

            std::string a;
	};

};

#endif

