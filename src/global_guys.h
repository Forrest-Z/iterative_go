#ifndef GLOBAL_GUYS_H
#define GLOBAL_GUYS_H

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <scanmatcher/scanmatcher.h>
#include <utils/point.h>

// using namespace std;

#define SQ(x) ((x)*(x))

namespace GMapping 
{
    class ScanMatcher; 
    // class OrientedPoint;
    // class ScanMatcherMap;
}

class CNodeScan;
class CICPWarpper;
// class CCanonicalMatcher;

// FOR DEBUG 
extern char int_info(const char* info = 0);

// FOR Olson's scan matcher method 
extern GMapping::ScanMatcher& getMatcher();
extern double ClimbHillMatcher(GMapping::OrientedPoint input, GMapping::OrientedPoint& output, GMapping::ScanMatcherMap* grid_map, double* r);

// FOR registering scan into gridmap
extern void addMapFromNode(CNodeScan*, GMapping::ScanMatcherMap*, GMapping::ScanMatcher&);

// FOR ICP PLICP
extern CICPWarpper* getICPMatcher();
// extern CCanonicalMatcher* getPLICPMatcher();

// FOR P1.ominus(p2)
extern GMapping::OrientedPoint ominus(GMapping::OrientedPoint p1, GMapping::OrientedPoint p2);
extern GMapping::OrientedPoint oplus(GMapping::OrientedPoint p1, GMapping::OrientedPoint p2);

// FOR loop closure and pose correction when rebuild the successive edge
extern std::vector<GMapping::OrientedPoint> interporlatePose(GMapping::OrientedPoint sp, GMapping::OrientedPoint ep, int n);


#endif
