#include "planner_modules_pr2/module_param_cache.h"
using namespace std;
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <sstream>
using namespace modules;

std::string createPoseParamString(const geometry_msgs::Pose & pose, double precPose, double precQuat)
{
    std::stringstream ss;
    ss.precision(0);
    ss << std::fixed;
    // the actual values don't matter as long as they are unique for this pose 
    // cannot use doubles here, as param keys are not allowed to contain '.' or '-'
    ss << "x";
    if(pose.position.x < 0)
        ss << "N";
    ss << abs(lrint(1.0/precPose * pose.position.x));
    ss << "y";
    if(pose.position.y < 0)
        ss << "N";
    ss << abs(lrint(1.0/precPose * pose.position.y));
    ss << "z";
    if(pose.position.z < 0)
        ss << "N";
    ss << abs(lrint(1.0/precPose * pose.position.z));

    ss << "qx";
    if(pose.orientation.x < 0)
        ss << "N";
    ss << abs(lrint(1.0/precQuat * pose.orientation.x));
    ss << "qy";
    if(pose.orientation.y < 0)
        ss << "N";
    ss << abs(lrint(1.0/precQuat * pose.orientation.y));
    ss << "qz";
    if(pose.orientation.z < 0)
        ss << "N";
    ss << abs(lrint(1.0/precQuat * pose.orientation.z));
    ss << "qw";
    if(pose.orientation.w < 0)
        ss << "N";
    ss << abs(lrint(1.0/precQuat * pose.orientation.w));

    return ss.str();
}

std::string computeFullStateCacheKey(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback)
{
    stringstream ss;
    ss << "FULL_S_PARAMS_";
    forEach(const Parameter & param, parameterList) {
        ss << param.value << "_";
    }
    ss << "___";

    NumericalFluentList* nfl = NULL;
    if(!numericalFluentCallback(nfl)) {
        ROS_ERROR("numericalFluentCallback failed.");
        return "";
    }

    ss << "NF";
    forEach(const NumericalFluent & nf, *nfl) {
        ss << nf.name << "_";
        forEach(const Parameter & param, nf.parameters) {
            ss << param.value << "_";
        }
        if(nf.value < 0)
            ss << "N";
        ss << abs(lrint(1000 * nf.value)) << "__";
    }
    delete nfl;

    PredicateList* pfl = NULL;
    if(!predicateCallback(pfl)) {
        ROS_ERROR("predicateCallback failed.");
        return "";
    }

    ss << "PF";
    forEach(const Predicate & pf, *pfl) {
        ss << pf.name << "_";
        forEach(const Parameter & param, pf.parameters) {
            ss << param.value << "_";
        }
        ss << pf.value << "__";
    }
    delete pfl;

    string key = ss.str();
    for(unsigned int i = 0; i < key.length(); i++) {
        if(key[i] == '-') {
            key[i] = '_';
        }
        if(key[i] == '!') {
            key[i] = 'X';
        }
        if(key[i] == '/') {
            key[i] = 'S';
        }
    }

    return key;
}

bool splitNamedId(const string & namedId, string & name, int & id)
{
    string::size_type foundIdx = namedId.find_first_of("0123456789");
    if(foundIdx == string::npos)
        return false;

    name = namedId.substr(0, foundIdx);

    std::stringstream ss(namedId.substr(foundIdx));
    ss >> id;
    return true;
}

