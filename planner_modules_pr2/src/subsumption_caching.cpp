#include "subsumption_caching.h"
#include <map>
#include <set>
using namespace std;
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <tf/tf.h>
#include <sstream>
#include <ros/ros.h>
#include "planner_modules_pr2/module_param_cache.h"

static const bool s_DebugSubsumption = false;

void encodeDouble(stringstream & ss, double d)
{
    ss.precision(0);
    if(d < 0)
        ss << "_N";
    ss << "_" << abs(lrint(10*1000*d));
}

double decodeDouble(stringstream & ss)
{
    // this holds either 'N' and the number (for negative)
    // or just the number
    double d;
    // First check if it's 'N'
    string s;
    ss >> s;
    if(s == "N") {    // next is actual number
        ss >> d;
        d /= -10*1000;
        if(d == 0) {  // make sure we get an N0 cache key from this.
          d = -0.000000001;
        }
        return d;
    }
    // s != 'N', re-interpret the token s as a double.
    stringstream ss2(s);
    ss2 >> d;
    d /= 10*1000;
    return d;
}

// encode a name for param caching (and later matching)
// For now basically drop all _
string encodeName(const string & s)
{
    string ret;
    for(unsigned int i = 0; i < s.size(); i++) {
        if(s[i] == '_')
            continue;
        ret += s[i];
    }
    return ret;
}

PutdownRequest::PutdownRequest() : computed(false)
{
}

PutdownRequest::PutdownRequest(const std::string & paramKey, const std::string & paramValue)
{
    // Stuff on param server is '_' separated as we can't store whitespace in keys,
    // change this to ' ' for stringstream parsing.
    string paramKeyConv = paramKey;
    for(unsigned int i = 0; i < paramKeyConv.size(); i++) {
        if(paramKeyConv[i] == '_')
            paramKeyConv[i] = ' ';
    }

    stringstream ss(paramKeyConv);
    string tokId;
    ss >> tokId;
    if(tokId != "PR") {
        ROS_ERROR("PutdownRequest from paramKey - Missing ID 'PR' - key is %s", paramKey.c_str());
    }
    ss >> putdownObject  >> putdownObjectType >> arm  >> staticObject;
    while(ss.good()) {
        MovableObject mo;
        // only check the first read, i.e. if we're at the end
        bool readOK = (ss >> mo.id);
        if(!readOK)
            break;
        // the rest should work, unless the data is corrupt
        ss >> mo.type;
        mo.pose.position.x = decodeDouble(ss);
        mo.pose.position.y = decodeDouble(ss);
        mo.pose.position.z = decodeDouble(ss);
        mo.pose.orientation.x = decodeDouble(ss);
        mo.pose.orientation.y = decodeDouble(ss);
        mo.pose.orientation.z = decodeDouble(ss);
        mo.pose.orientation.w = decodeDouble(ss);
        objectsOnStatic.push_back(mo);
    }

    if(paramValue.empty()) {
        computed = false;
        return;
    }

    stringstream ssVal(paramValue);
    ssVal >> computed;
    ssVal >> computationTime;
    ssVal >> success;
    ssVal >> putdownPose.position.x;
    ssVal >> putdownPose.position.y;
    ssVal >> putdownPose.position.z;
    ssVal >> putdownPose.orientation.x;
    ssVal >> putdownPose.orientation.y;
    ssVal >> putdownPose.orientation.z;
    ssVal >> putdownPose.orientation.w;
}

std::string PutdownRequest::toParamKey() const
{
    stringstream ss;
    ss << fixed;
    ss << "PR_" << encodeName(putdownObject) << "_" << encodeName(putdownObjectType) << "_"
        << encodeName(arm) << "_" << encodeName(staticObject);

    forEach(const MovableObject & mo, objectsOnStatic) {
        ss << "_" << encodeName(mo.id) << "_" << encodeName(mo.type);
        encodeDouble(ss, mo.pose.position.x);
        encodeDouble(ss, mo.pose.position.y);
        encodeDouble(ss, mo.pose.position.z);
        encodeDouble(ss, mo.pose.orientation.x);
        encodeDouble(ss, mo.pose.orientation.y);
        encodeDouble(ss, mo.pose.orientation.z);
        encodeDouble(ss, mo.pose.orientation.w);
    }
    return ss.str();
}

std::string PutdownRequest::toParamValue() const
{
    stringstream ss;
    ss << (computed ? "1" : "0");
    ss << " " << computationTime;
    ss << " " << (success ? "1" : "0");
    ss << " " << putdownPose.position.x;
    ss << " " << putdownPose.position.y;
    ss << " " << putdownPose.position.z;
    ss << " " << putdownPose.orientation.x;
    ss << " " << putdownPose.orientation.y;
    ss << " " << putdownPose.orientation.z;
    ss << " " << putdownPose.orientation.w;
    return ss.str();
}


SubsumptionCache::SubsumptionCache(const std::string & moduleNamespace) :
    moduleNamespace(moduleNamespace)
{
}

void SubsumptionCache::put(const PutdownRequest & pdr)
{
    if(!pdr.computed) {
        ROS_ERROR("SubsumptionCache::put got a request that wasn't computed.");
        return;
    }

    ROS_INFO("SubsumptionCache adding request.");
    if(s_DebugSubsumption) {
        cout << "Before put" << endl;
        dump();
    }

    // Check for subsumptions depending on the kind of request
    // If there is a match:
    // Either we might not need to insert this at is is subsumed or
    // we can insert this and remove another one.
    unsigned int replaceCount = 0;
    if(pdr.success) {
        forEach(PutdownRequest & suc, successfullRequests) {
            // check if pdr is more constrained than suc
            // If it is, we have found something even harder to be solved.
            if(isMoreConstrained(pdr, suc) == 1) {
                suc = pdr;
                replaceCount++;
            }
            // FIXME: we should check that we replace only once and remove extras
            // as then we'd have the same request in there doubly.
        }
        if(replaceCount == 0)
            successfullRequests.push_back(pdr);
        else
            ROS_INFO("Subsumed a SUCCESS request.");
    } else {    // pdr is fail
        forEach(PutdownRequest & fail, failedRequests) {
            // check if fail is more constrained than pdr
            // If it is, we have found something even easier that failed.
            if(isMoreConstrained(fail, pdr) == 1) {
                fail = pdr;
                replaceCount++;
            }
            // FIXME: we should check that we replace only once and remove extras
            // as then we'd have the same request in there doubly.
        }
        if(replaceCount == 0)
            failedRequests.push_back(pdr);
        else
            ROS_INFO("Subsumed a FAIL request.");
    }

    if(replaceCount > 1) {
        ROS_WARN("Got a request that could be subsumed more than once.");
    }

    if(s_DebugSubsumption) {
        cout << "After put" << endl;
        dump();
    }
}

bool SubsumptionCache::query(const PutdownRequest & pdr, PutdownRequest & match)
{
    // Let's see if the pdr is subsumed by another request and return than.
    forEach(const PutdownRequest & suc, successfullRequests) {
        int constr = isMoreConstrained(suc, pdr);
        if(constr == 0) {   // exact match
            match = suc;
            return true;
        }
        if(constr == 1) {   // suc is even more constrained than pdr
            match = suc;
            return true;
        }
    }
    forEach(const PutdownRequest & fail, failedRequests) {
        int constr = isMoreConstrained(pdr, fail);
        if(constr == 0) {   // exact match
            match = fail;
            return true;
        }
        if(constr == 1) {   // pdr is even more constrained than fail
            match = fail;
            return true;
        }
    }

    // otherwise, we just write it back.
    match = pdr;
    return false;
}

void SubsumptionCache::debugQuery(const PutdownRequest & pdr) const
{
    bool ret = false;
    PutdownRequest match;

    ROS_INFO_STREAM("Debug query for request:" << endl << pdr);

    ROS_INFO("Checking cached successes:");
    // Let's see if the pdr is subsumed by another request and return than.
    forEach(const PutdownRequest & suc, successfullRequests) {
        int constr = isMoreConstrained(suc, pdr);
        ROS_INFO_STREAM("PDR: " << suc << "isMoreConstrained: " << constr << endl);
        if(constr == 0) {   // exact match
            if(!ret)            // record the first, that's where query() would have quit.
                match = suc;
            ROS_INFO("Exact match");
            ret = true;
        }
        if(constr == 1) {   // suc is even more constrained than pdr
            if(!ret)
                match = suc;
            ROS_INFO("Subsumption match");
            ret = true;
        }
    }
    ROS_INFO_STREAM("Checking cached failures:");
    forEach(const PutdownRequest & fail, failedRequests) {
        int constr = isMoreConstrained(pdr, fail);
        ROS_INFO_STREAM("PDR: " << fail << "isMoreConstrained: " << constr << endl);
        if(constr == 0) {   // exact match
            if(!ret)
                match = fail;
            ROS_INFO("Exact match");
            ret = true;
        }
        if(constr == 1) {   // pdr is even more constrained than fail
            if(!ret)
                match = fail;
            ROS_INFO("Subsumption match");
            ret = true;
        }
    }

    // otherwise, we just write it back.
    if(ret) {
        ROS_INFO_STREAM("Found matching PDR: " << match);
    } else {
        ROS_INFO_STREAM("Could not find matching PDR.");
    }
}

int SubsumptionCache::isMoreConstrained(const PutdownRequest & lhs, const PutdownRequest & rhs) const
{
    if(encodeName(lhs.putdownObjectType) != encodeName(rhs.putdownObjectType))
        return -100;
    if(encodeName(lhs.arm) != encodeName(rhs.arm))
        return -100;
    if(encodeName(lhs.staticObject) != encodeName(rhs.staticObject))
        return -100;

    // match the objectsOnStatic of both.

    // first figure out which objects are matching, i.e. the same in both requests
    set<string> unmatchedObjectsLhs;
    set<string> unmatchedObjectsRhs;
    forEach(const PutdownRequest::MovableObject & mo, lhs.objectsOnStatic) {
        unmatchedObjectsLhs.insert(mo.id);
    }
    forEach(const PutdownRequest::MovableObject & mo, rhs.objectsOnStatic) {
        unmatchedObjectsRhs.insert(mo.id);
    }

    forEach(const PutdownRequest::MovableObject & mol, lhs.objectsOnStatic) {
        forEach(const PutdownRequest::MovableObject & mor, rhs.objectsOnStatic) {
            if(objectMatches(mol, mor)) {
                unmatchedObjectsLhs.erase(mol.id);
                unmatchedObjectsRhs.erase(mor.id);
            }
        }
    }

    // From the remaining objects determine if we have a constraint.
    // If there are remaining differing objects in both requests, this cannot be
    // matched.
    if(!unmatchedObjectsLhs.empty() && !unmatchedObjectsRhs.empty())
        return -100;
    if(!unmatchedObjectsLhs.empty() && unmatchedObjectsRhs.empty())
        return 1;
    if(unmatchedObjectsLhs.empty() && !unmatchedObjectsRhs.empty())
        return -1;

    // no unmatched objects for both
    return 0;
}

bool SubsumptionCache::objectMatches(const PutdownRequest::MovableObject & mol, const PutdownRequest::MovableObject & mor) const
{
    if(encodeName(mol.type) != encodeName(mor.type))
        return false;

    // using createPoseParamString here although thresholds would be nicer
    // to be comparable to partial state caching
    string poseStrL = createPoseParamString(mol.pose);
    string poseStrR = createPoseParamString(mor.pose);
    //printf("objectMatches:\n%s\n%s\n= %d\n", poseStrL.c_str(), poseStrR.c_str(), (poseStrL == poseStrR));

    return poseStrL == poseStrR;

#if 0
    // pose matches
    tf::Pose tfl, tfr;
    tf::poseMsgToTF(mol.pose, tfl);
    tf::poseMsgToTF(mor.pose, tfr);
    tf::Pose tfDelta = tfl.inverseTimes(tfr);

    // need an exact match to be fair to parital state caching

    // distance between poses
    if(tfDelta.getOrigin().length() > 0.001)
        return false;

    if(tfDelta.getRotation().getAngle() > 0.5 * M_PI / 180.0)
        return false;

    return true;
#endif
}

std::string SubsumptionCache::getParamKeyPrefix()
{
    ros::NodeHandle nh;

    std::string runName;
    if(nh.hasParam("/tfd_modules/run_name")) {
        nh.getParam("/tfd_modules/run_name", runName);
    }
    // /tfd_modules/module_cache/RUN_NAME/PUTDOWN_STUFF
    std::string baseNamespace = "/tfd_modules/module_cache/";
    std::string keyPrefix = baseNamespace;
    if(!runName.empty())
        keyPrefix += runName + "/";
    if(!moduleNamespace.empty())
        keyPrefix += moduleNamespace + "/";
    return keyPrefix;
}

void SubsumptionCache::writeToParams()
{
    ros::NodeHandle nh;
    string keyPrefix = getParamKeyPrefix();

    // need to clear the path first, as some older ones might be
    // subsumed now and are merely overhead.
    if(nh.hasParam(keyPrefix)) {
        string cmd = "rosparam delete " + keyPrefix;
        int ret = system(cmd.c_str());
        if(ret != 0) {
            ROS_WARN("%s: Deleting old params failed at: %s (err: %d)", __func__, keyPrefix.c_str(), ret);
        }
    }

    // now write the cache
    forEach(const PutdownRequest & pr, successfullRequests) {
        nh.setParam(keyPrefix + pr.toParamKey(), pr.toParamValue());
    }
    forEach(const PutdownRequest & pr, failedRequests) {
        nh.setParam(keyPrefix + pr.toParamKey(), pr.toParamValue());
    }
}

bool SubsumptionCache::readFromParams()
{
    if(s_DebugSubsumption) {
        cout << "Reading SubsumptionCache from params" << endl;
    }
    successfullRequests.clear();
    failedRequests.clear();

    ros::NodeHandle nh;
    string keyPrefix = getParamKeyPrefix();

    // need to read in the dict at keyPrefix
    XmlRpc::XmlRpcValue xmlRpc;
    if(!nh.getParam(keyPrefix, xmlRpc)) {
        ROS_WARN("%s: Could not get xml rpc value from %s", __func__, keyPrefix.c_str());
        return false;
    }

    if(xmlRpc.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("%s: xmlrpc is not a dict", __func__);
        return false;
    }

    bool ok = true;
    for(XmlRpc::XmlRpcValue::iterator it = xmlRpc.begin(); it != xmlRpc.end(); it++) {
        string paramKey = it->first;
        // value: XmlRpcValue
        XmlRpc::XmlRpcValue paramValXmlRpc = it->second;
        if(paramValXmlRpc.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("%s: param value not a string for: %s", __func__, paramKey.c_str());
            ok = false;
            continue;
        }
        string paramValue = (string)paramValXmlRpc;

        PutdownRequest pr(paramKey, paramValue);
        if(!pr.computed) {
            ROS_ERROR("%s: non-computed PutdownRequest at %s", __func__, paramKey.c_str());
            ok = false;
            continue;
        }
        if(pr.success)
            successfullRequests.push_back(pr);
        else
            failedRequests.push_back(pr);
    }

    if(s_DebugSubsumption) {
        cout << "SubsumptionCache: read from params" << endl;
        dump();
    }

    return ok;
}

void SubsumptionCache::dump() const
{
    cout << "SubsumptionCache" << endl;
    cout << "successfullRequests (" << successfullRequests.size() << ")" << endl;
    forEach(const PutdownRequest & pdr, successfullRequests) {
        cout << pdr;
    }
    cout << "failedRequests (" << failedRequests.size() << ")" << endl;
    forEach(const PutdownRequest & pdr, failedRequests) {
        cout << pdr;
    }
    cout << endl;
}

std::ostream & operator<<(std::ostream & os, const PutdownRequest::MovableObject & mo)
{
    os << "MovableObject " << mo.id << " (" << mo.type << ") at " << mo.pose;
    return os;
}

std::ostream & operator<<(std::ostream & os, const PutdownRequest & pdr)
{
    os << "PutdownRequest for " << pdr.putdownObject << " (" << pdr.putdownObjectType << ") in "
        << pdr.arm << " to " << pdr.staticObject << endl;

    os << "Objects on same static:";
    forEach(const PutdownRequest::MovableObject & mo, pdr.objectsOnStatic) {
        os << mo << endl;
    }

    if(pdr.computed) {
        os << "Took " << pdr.computationTime << " s to compute: result: ";
        if(pdr.success) {
            os << "success, putdown pose: " << pdr.putdownPose;
        } else {
            os << "failure";
        }
    } else {
        os << "Not computed";
    }
    os << endl;
    return os;
}

