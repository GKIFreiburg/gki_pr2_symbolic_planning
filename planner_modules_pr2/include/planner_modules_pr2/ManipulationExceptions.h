/*
 * ManipulationExceptions.h
 *
 *  Created on: Jul 29, 2015
 *      Author: andreas
 */

#ifndef MANIPULATIONEXCEPTIONS_H_
#define MANIPULATIONEXCEPTIONS_H_

#include <exception>
#include <string>
#include <sstream>
#include <moveit_msgs/MoveItErrorCodes.h>

namespace planner_modules_pr2
{

struct ManipulationException : public virtual std::exception
{
};

struct PickupPlanFailedException : public virtual ManipulationException
{
	moveit_msgs::MoveItErrorCodes::_val_type error_value;
	PickupPlanFailedException(const moveit_msgs::MoveItErrorCodes& error_code) : ManipulationException(), error_value(error_code.val)
	{
	}

	virtual const char* what() const throw()
	{
		std::stringstream buffer;
		moveit_msgs::MoveItErrorCodes error;
		error.val = error_value;
		buffer << "MoveIt error code: " << error;
		return buffer.str().c_str();
	}
};

struct ObjectNotFoundInSceneException : public virtual ManipulationException
{
	std::string id;
	ObjectNotFoundInSceneException(const std::string& id) : ManipulationException(), id(id)
	{
	}
	virtual ~ObjectNotFoundInSceneException() throw()
	{
	}

	virtual const char* what() const throw()
	{
		std::stringstream buffer;
		buffer << "Cannot find object " << id << " in scene.";
		return buffer.str().c_str();
	}
};

struct GeneratingGraspsTimeoutException : public virtual ManipulationException
{
	const char* what() const throw()
	{
		return "Timeout while generating grasps.";
	}
};

struct GeneratingGraspsFailedException : public ManipulationException
{
	virtual const char* what() const throw()
	{
		return "Generating grasps failed.";
	}
};

struct ZeroGraspsGeneratedException : public ManipulationException
{
	virtual const char* what() const throw()
	{
		return "No grasps were generated.";
	}
};

} /* namespace planner_modules_pr2 */

#endif /* MANIPULATIONEXCEPTIONS_H_ */
