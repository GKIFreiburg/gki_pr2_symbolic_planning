#ifndef ACTION_EXECUTOR_PUTDOWN_OBJECT_H
#define ACTION_EXECUTOR_PUTDOWN_OBJECT_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"

#include <symbolic_planning_utils/planning_scene_interface.h>
#include <object_surface_placements/placement_generator.h>
#include <object_surface_placements/object_surface_placements.h>
//#include <tidyup_msgs/PlaceObjectAction.h>

namespace object_manipulation_actions
{

	class PutdownObject: public continual_planning_executive::ActionExecutorInterface
	{
    	public:

		PutdownObject();
		~PutdownObject();

		virtual void initialize(const std::deque<std::string> & arguments);

		virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

		virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

		virtual void cancelAction();

    	private:

		std::string action_name_;
		std::string predicate_inspected_recently_;
		boost::shared_ptr<symbolic_planning_utils::PlanningSceneInterface> psi_;
		boost::shared_ptr<object_surface_placements::PlacementGenerator> placement_gen_;

		object_surface_placements::CollisionMethod collision_method_;
		double z_above_table_;

	};

};

#endif // ACTION_EXECUTOR_PUTDOWN_OBJECT_H

