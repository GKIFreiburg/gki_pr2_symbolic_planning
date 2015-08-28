#include "planner_modules_pr2/module_param_cache.h"
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <sstream>

namespace planner_modules_pr2
{
using namespace std;
using namespace modules;
inline void writeKeyDouble(std::stringstream& ss, const string& name, double value, double precision = 0.01)
{
	ss << name;
	if (value < 0)
	{
		ss << "N";
	}
	ss << abs(lrint(1.0 / precision * value));
}

std::string createPoseParamString(const geometry_msgs::Pose& pose, double precPose, double precQuat)
{
	std::stringstream ss;
	ss.precision(0);
	ss << std::fixed;
	// the actual values don't matter as long as they are unique for this pose
	// cannot use doubles here, as param keys are not allowed to contain '.' or '-'
	writeKeyDouble(ss, "x", pose.position.x, precPose);
	writeKeyDouble(ss, "y", pose.position.y, precPose);
	writeKeyDouble(ss, "z", pose.position.z, precPose);
	writeKeyDouble(ss, "qx", pose.orientation.x, precQuat);
	writeKeyDouble(ss, "qy", pose.orientation.y, precQuat);
	writeKeyDouble(ss, "qz", pose.orientation.z, precQuat);
	writeKeyDouble(ss, "qw", pose.orientation.w, precQuat);

	return ss.str();
}

std::string createPoseParamString(const geometry_msgs::Pose2D& pose, double torso_position, double precPose, double precTheta)
{
	std::stringstream ss;
	ss.precision(0);
	ss << std::fixed;
	// the actual values don't matter as long as they are unique for this pose
	// cannot use doubles here, as param keys are not allowed to contain '.' or '-'
	writeKeyDouble(ss, "x", pose.x, precPose);
	writeKeyDouble(ss, "y", pose.y, precPose);
	writeKeyDouble(ss, "th", pose.theta, precTheta);
	writeKeyDouble(ss, "tp", torso_position, precPose);

	return ss.str();
}

bool splitNamedId(const string & namedId, string & name, int & id)
{
	string::size_type foundIdx = namedId.find_first_of("0123456789");
	if (foundIdx == string::npos)
		return false;

	name = namedId.substr(0, foundIdx);

	std::stringstream ss(namedId.substr(foundIdx));
	ss >> id;
	return true;
}

}
