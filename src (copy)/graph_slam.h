/*
    Author: David Z.H. Feb. 7th 2014
*/


#ifndef GRAPH_SLAM_H
#define GRAPH_SLAM_H

#include <map>
#include <sensor/sensor_range/rangesensor.h>
#include <sensor/sensor_range/rangereading.h>
#include <scanmatcher/scanmatcher.h>
#include <gridfastslam/motionmodel.h>
#include "node_scan.h"
#include "macro_params.h"
// #include "ZHIcp_Warpper.h"

using namespace std;

class CICPWarpper; 

namespace g2o{
    class SparseOptimizer; 
    class SparseOptimizerOnline;
    class EdgeSE2;
}

class MatchingResult;


class CGraph_SLAM
{
public:
    CGraph_SLAM(double minScore = 0 , int nParticles = 10);
    ~CGraph_SLAM();
 
    GMapping::OrientedPoint getCurrPose(){return m_pose_curr;}
    unsigned int size(){return m_graph.size();}
    void writePath(const char* );

    bool addNode(CNodeScan * );
    void setSensorMap(GMapping::SensorMap& );
    void init(double xmin, double ymin, double xmax, double ymax, double delta,
            GMapping::OrientedPoint initialPose = GMapping::OrientedPoint(0,0,0));
    void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma=1, double likelihoodGain=1, unsigned int likelihoodSkip=0);
    void setMotionModelParameters(double srr, double srt, double str, double stt);
    void setGenerateMap(bool );
    
    double linearUpdate_;
    double angularUpdate_;
    PARAM_SET_GET(double, linearUpdate, protected, public, public);
    PARAM_SET_GET(double, angularUpdate, protected, public, public);

    map<int, CNodeScan*> m_graph; 
    typedef map<int, CNodeScan*>::iterator iterator;
    iterator begin(){return m_graph.begin();}
    iterator end(){return m_graph.end();}

    // motion-model 
    GMapping::MotionModel m_motionModel;

    // scan-matcher using climb-hill 
    GMapping::OrientedPoint MRMC_ClimbHill(CNodeScan*);
    GMapping::OrientedPoint m_pose_odo_now; 
    GMapping::OrientedPoint m_pose_odo_pri;
    GMapping::OrientedPoint m_pose_last; 
    GMapping::OrientedPoint m_pose_curr;
    GMapping::ScanMatcher m_matcher; 
    
    // grid map
    GMapping::ScanMatcherMap * m_gridmap;
    void getGridMap(GMapping::ScanMatcher& matcher, GMapping::ScanMatcherMap* gridmap);
    
    // ICP Pairwise
    CICPWarpper* m_icp;
    GMapping::OrientedPoint ICP_PairWise(CNodeScan*, GMapping::OrientedPoint&);
    PARAM_SET_GET(double, icpGoodness, protected, public, public);

    // G2O
public:
    bool addEdgeToG2O(MatchingResult& , bool large_edge, bool set_estimate); 
    inline GMapping::OrientedPoint ominus(GMapping::OrientedPoint& p1, GMapping::OrientedPoint& p2);
    void fromOrientedPoint2SE2(GMapping::OrientedPoint& trans, g2o::EdgeSE2& ret);
    void saveG2O(const char* fname = 0);
    #ifndef ONLINE
        g2o::SparseOptimizer * optimizer_; 
    #else
        g2o::SparseOptimizerOnline * optimizer_;
    #endif 

protected:
    void addMapFromNode(CNodeScan*, GMapping::ScanMatcherMap*, GMapping::ScanMatcher&);
    void setParticlePoses(GMapping::OrientedPoint&);
    int m_nMRMC; // number of particles to predicate motion using climb-hill 
    double m_minScore; // minScore for climb-hill
    vector<GMapping::OrientedPoint> m_nposes;
    // int m_count; // number of nodes   
};


#endif
