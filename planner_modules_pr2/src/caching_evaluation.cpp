#include "caching_evaluation.h"
#include <utility>
using namespace std;
#include <ros/ros.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

CachingEntry::CachingEntry(double time) : computationTime(time)
{
    // this is a new entry, so this must be a miss
    hits = 0;
    misses = 1;
}

CachingEntry::CachingEntry(bool hit, double time) : computationTime(time)
{
    if(hit) {
        hits = 1;
        misses = 0;
    } else {
        hits = 0;
        misses = 1;
    }
}

CachingEvaluation::CachingEvaluation()
{
    outputFilename = "caching_results.txt";
}

void CachingEvaluation::recordNoCacheQuery(std::string & bestCacheKey, double time)
{
    map<string, CachingEntry>::iterator it = noCacheCachings.find(bestCacheKey);
    if(it == noCacheCachings.end()) {
        noCacheCachings.insert(make_pair(bestCacheKey, CachingEntry(time)));
    } else {
        it->second.misses++;
    }
}

void CachingEvaluation::recordFullCacheQueryGlobal(const std::string & bestCacheKey,
        bool hit, double time)
{
    map<string, CachingEntry>::iterator it = fullStateCachings.find(bestCacheKey);
    if(it == fullStateCachings.end()) {
        fullStateCachings.insert(make_pair(bestCacheKey, CachingEntry(hit, time)));
    } else {
        if(hit)
            it->second.hits++;
        else
            it->second.misses++;
    }
}

void CachingEvaluation::recordPartialCacheQueryGlobal(const std::string & bestCacheKey,
        bool hit, double time)
{
    map<string, CachingEntry>::iterator it = partialStateCachings.find(bestCacheKey);
    if(it == partialStateCachings.end()) {
        partialStateCachings.insert(make_pair(bestCacheKey, CachingEntry(hit, time)));
    } else {
        if(hit)
            it->second.hits++;
        else
            it->second.misses++;
    }
}

void CachingEvaluation::recordFullCacheQueryLocal(const std::string & bestCacheKey,
        bool hit, double time)
{
    map<string, CachingEntry>::iterator it = fullStateLocalCachings.find(bestCacheKey);
    if(it == fullStateLocalCachings.end()) {
        fullStateLocalCachings.insert(make_pair(bestCacheKey, CachingEntry(hit, time)));
    } else {
        if(hit)
            it->second.hits++;
        else
            it->second.misses++;
    }
}

void CachingEvaluation::recordPartialCacheQueryLocal(const std::string & bestCacheKey,
        bool hit, double time)
{
    map<string, CachingEntry>::iterator it = partialStateLocalCachings.find(bestCacheKey);
    if(it == partialStateLocalCachings.end()) {
        partialStateLocalCachings.insert(make_pair(bestCacheKey, CachingEntry(hit, time)));
    } else {
        if(hit)
            it->second.hits++;
        else
            it->second.misses++;
    }
}

void CachingEvaluation::recordSubsumedCacheQueryGlobal(const std::string & bestCacheKey,
        bool hit, double time)
{
    map<string, CachingEntry>::iterator it = subsumedStateCachings.find(bestCacheKey);
    if(it == subsumedStateCachings.end()) {
        subsumedStateCachings.insert(make_pair(bestCacheKey, CachingEntry(hit, time)));
    } else {
        if(hit)
            it->second.hits++;
        else
            it->second.misses++;
    }
}

void CachingEvaluation::recordSubsumedCacheQueryLocal(const std::string & bestCacheKey,
        bool hit, double time)
{
    map<string, CachingEntry>::iterator it = subsumedStateLocalCachings.find(bestCacheKey);
    if(it == subsumedStateLocalCachings.end()) {
        subsumedStateLocalCachings.insert(make_pair(bestCacheKey, CachingEntry(hit, time)));
    } else {
        if(hit)
            it->second.hits++;
        else
            it->second.misses++;
    }
}

void CachingEvaluation::writeResults()
{
    ofstream resultsFile(outputFilename.c_str());
    if(!resultsFile.good()) {
        ROS_ERROR("%s: Could not open %s", __PRETTY_FUNCTION__, outputFilename.c_str());
        return;
    }

    unsigned int ncHits = 0;
    unsigned int ncMisses = 0;
    double ncVirtualTime = 0.0;
    for(map<string,CachingEntry>::iterator it = noCacheCachings.begin(); it != noCacheCachings.end(); it++) {
        ncHits += it->second.hits;
        ncMisses += it->second.misses;
        ncVirtualTime += it->second.misses * it->second.computationTime;
    }
    resultsFile << "NoCache " << ncHits << " " << ncMisses << " " << ncVirtualTime << endl;

    unsigned int fsHits = 0;
    unsigned int fsMisses = 0;
    double fsVirtualTime = 0.0;
    for(map<string,CachingEntry>::iterator it = fullStateCachings.begin(); it != fullStateCachings.end(); it++) {
        fsHits += it->second.hits;
        fsMisses += it->second.misses;
        fsVirtualTime += it->second.misses * it->second.computationTime;
    }
    resultsFile << "FullStateGlobal " << fsHits << " " << fsMisses << " " << fsVirtualTime << endl;

    unsigned int psHits = 0;
    unsigned int psMisses = 0;
    double psVirtualTime = 0.0;
    for(map<string,CachingEntry>::iterator it = partialStateCachings.begin(); it != partialStateCachings.end(); it++) {
        psHits += it->second.hits;
        psMisses += it->second.misses;
        psVirtualTime += it->second.misses * it->second.computationTime;
    }
    resultsFile << "PartialStateGlobal " << psHits << " " << psMisses << " " << psVirtualTime << endl;

    unsigned int ssHits = 0;
    unsigned int ssMisses = 0;
    double ssVirtualTime = 0.0;
    for(map<string,CachingEntry>::iterator it = subsumedStateCachings.begin(); it != subsumedStateCachings.end(); it++) {
        ssHits += it->second.hits;
        ssMisses += it->second.misses;
        ssVirtualTime += it->second.misses * it->second.computationTime;
    }
    resultsFile << "SubsumedStateGlobal " << ssHits << " " << ssMisses << " " << ssVirtualTime << endl;


    unsigned int fsLocalHits = 0;
    unsigned int fsLocalMisses = 0;
    double fsLocalVirtualTime = 0.0;
    for(map<string,CachingEntry>::iterator it = fullStateLocalCachings.begin(); it != fullStateLocalCachings.end(); it++) {
        fsLocalHits += it->second.hits;
        fsLocalMisses += it->second.misses;
        fsLocalVirtualTime += it->second.misses * it->second.computationTime;
    }
    resultsFile << "FullStateLocal " << fsLocalHits << " " << fsLocalMisses << " " << fsLocalVirtualTime << endl;

    unsigned int psLocalHits = 0;
    unsigned int psLocalMisses = 0;
    double psLocalVirtualTime = 0.0;
    for(map<string,CachingEntry>::iterator it = partialStateLocalCachings.begin(); it != partialStateLocalCachings.end(); it++) {
        psLocalHits += it->second.hits;
        psLocalMisses += it->second.misses;
        psLocalVirtualTime += it->second.misses * it->second.computationTime;
    }
    resultsFile << "PartialStateLocal " << psLocalHits << " " << psLocalMisses << " " << psLocalVirtualTime << endl;

    unsigned int ssLocalHits = 0;
    unsigned int ssLocalMisses = 0;
    double ssLocalVirtualTime = 0.0;
    for(map<string,CachingEntry>::iterator it = subsumedStateLocalCachings.begin(); it != subsumedStateLocalCachings.end(); it++) {
        ssLocalHits += it->second.hits;
        ssLocalMisses += it->second.misses;
        ssLocalVirtualTime += it->second.misses * it->second.computationTime;
    }
    resultsFile << "SubsumedStateLocal " << ssLocalHits << " " << ssLocalMisses << " " << ssLocalVirtualTime << endl;

    resultsFile.close();
}

