#ifndef STATE_CREATOR_ARMS_AT_SIDE_H
#define STATE_CREATOR_ARMS_AT_SIDE_H

#include "continual_planning_executive/stateCreator.h"
#include <ros/ros.h>
#include <tidyup_msgs/ArmsAtSide.h>

namespace tidyup_state_creators
{

    /// This state creator adds the arm-at-side predicate for both arms
    class StateCreatorArmsAtSide : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorArmsAtSide();
            ~StateCreatorArmsAtSide();

            /// Initialize the state creator parameters.
            /**
             * args: service_name predicate_name predicate_value
             *
             * The service name of the arms-at-side service.
             */
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

        protected:
            ros::ServiceClient client;
            string service_name;
            string predicate_name;
            string predicate_value;
            string unknown_value;
    };

};

#endif

