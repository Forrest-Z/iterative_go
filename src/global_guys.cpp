#include "global_guys.h"
#include <gridfastslam/motionmodel.h>
#include "node_scan.h"
#include "ros/ros.h"
#include "ZHIcp_Warpper.h"
// #include "ZHCanonical_Matcher.h"
// #include "PolarParameter.h"
#include "point.h"

using namespace std;

char int_info(const char* info )
{
    if(info == 0)
        cout<<"interupt here, please input a key to continue;"<<endl;
    else
        cout<<info<<endl;
    char k;
    cin>>k;
    return k;
}

GMapping::ScanMatcher& getMatcher()
{
    static GMapping::ScanMatcher matcher; 
    return matcher;
}

/*CCanonicalMatcher* getPLICPMatcher()
{
    static CCanonicalMatcher* pPLICP = 0;
    if(pPLICP == 0)
    {
        Base_PARAM* param = ObtainParam<_PM_SICK_LMS511>();
        pPLICP = new CCanonicalMatcher(param);
    }
    return pPLICP;
}*/

CICPWarpper* getICPMatcher()
{
    static CICPWarpper* pICP = 0; 
    if(pICP == 0)
    {
        pICP = new CICPWarpper;
        pICP->initICP();
    }
    return pICP;
}

void addMapFromNode(CNodeScan* new_node, GMapping::ScanMatcherMap* gridmap, GMapping::ScanMatcher& matcher)
{
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(*gridmap, *(new_node->getPose()), new_node->getPlainReading());
    // matcher.invalidateActiveArea();
    matcher.registerScan(*gridmap,  *(new_node->getPose()), new_node->getPlainReading());
}

GMapping::OrientedPoint ominus(GMapping::OrientedPoint p1, GMapping::OrientedPoint p2)
{
    OrientedPoint2D pd1(p1.x, p1.y, p1.theta); 
    OrientedPoint2D pd2(p2.x, p2.y, p2.theta); 
    OrientedPoint2D rel = pd1.ominus(pd2);
    return GMapping::OrientedPoint(rel.x, rel.y, rel.theta);
}

GMapping::OrientedPoint oplus(GMapping::OrientedPoint p1, GMapping::OrientedPoint p2)
{
    OrientedPoint2D pd1(p1.x, p1.y, p1.theta); 
    OrientedPoint2D rel(p2.x, p2.y, p2.theta); 
    OrientedPoint2D pd2 = pd1.oplus(rel);
    return GMapping::OrientedPoint(pd2.x, pd2.y, pd2.theta);
}


double ClimbHillMatcher(GMapping::OrientedPoint in, GMapping::OrientedPoint& out, GMapping::ScanMatcherMap* grid_map, double * r)
{
    double score = getMatcher().optimize(out, *grid_map, in, r);
    return score;
}

vector<GMapping::OrientedPoint> interporlatePose(GMapping::OrientedPoint sp, GMapping::OrientedPoint ep, int n)
{
    vector<GMapping::OrientedPoint> ret_p(n); 
    float sx, sy, stheta; 
    float ex, ey, etheta; 
    float dx, dy, dtheta; 
    sx = sp.x; sy = sp.y; stheta = sp.theta; 
    ex = ep.x; ey = ep.y; etheta = ep.theta;
    dx = (ex-sx)/(float)(n); 
    dy = (ey-sy)/(float)(n); 
    dtheta = (etheta-stheta)/(float)(n);
    for(int i=0; i<n; i++)
    {
        ret_p[i] = GMapping::OrientedPoint(sx + dx*i, sy + dy*i, stheta + dtheta*i);

        // until now, not theta 
        ret_p[i].theta = stheta; 
    }
    return ret_p;
}

