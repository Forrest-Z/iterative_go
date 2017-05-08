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
#include "key_node.h"
#include "macro_params.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
// #include "ZHIcp_Warpper.h"
#include "global_guys.h"

using namespace std;

class CICPWarpper; 

namespace g2o
{
    class SparseOptimizer; 
    class SparseOptimizerOnline;
    class EdgeSE2;
}

class MatchingResult;

class CGraph_SLAM
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CGraph_SLAM(double minScore = 295 , int nParticles = 30);
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
    
    // whether to use laser odometry
    bool use_laser_odo_;
    vector<float> prev_x_;
    vector<float> prev_y_;
    vector<double> prev_r_;
    // CNodeScan* prev_node_; // used to estimate ICP alignment
    
    // check whether this node has been updated by loop 
    vector<bool> m_loop_node;

    // score check for rebuild map 
    map<int, float> m_score; 
    bool exceedScore(int id, float score);
    float scoreWeight(float s);
    float m_score_scale; // 0.7
    
    // cov check for rebuild map 
    map<int, vector<double> > m_cov; // covariance for each pose, used for constriant when rebuilding map
    void recordCov(int id); // record cov for this id
    bool exceedCov(GMapping::OrientedPoint disp, int id);
    vector<GMapping::OrientedPoint> checkPoseCov(vector<GMapping::OrientedPoint>&, int id );
    double m_cov_motion_scale; // numeric 1.737
    double m_cov_angle_scale; // numeric 3
    double m_CH_step; // Climbhill step 
    int m_extra_samples; // 

    // all nodes
    map<int, CNodeScan*> m_graph; 
    typedef map<int, CNodeScan*>::iterator iterator;
    iterator begin(){return m_graph.begin();}
    iterator end(){return m_graph.end();}

    // key nodes 
    multimap<int, int> m_key_loop; // 
    void rebuildLoopEdge();

    map<int, CKeyNode*> m_key_graph; 
    typedef map<int, CKeyNode*>::iterator key_iterator; 
    key_iterator kbegin(){return m_key_graph.begin();}
    key_iterator kend(){return m_key_graph.end();}
    CKeyNode* current_key_node_;
    std::vector<int> getPotentialLoopKeyNodes(CNodeScan*);
    double sqrDisPose(GMapping::OrientedPoint*, GMapping::OrientedPoint*);
    double sqr_dis_thresh_; // whether to start a new key_node
    void registerKeyNode(CKeyNode*, bool breset = false);
   
    void reconstructMap(); // only reconstruct according to current G2O value
    void rebuildMapAndG2O(); // rebuild Map and G2O from raw reading and G2O update
    void reconstructKeyMap(); // as it says, reconstruct key_node's map
    void reconstructG2O(); // replace the value of edge using scan matching

    // motion-model 
    GMapping::MotionModel m_motionModel;
    Eigen::Matrix3d odo_cov_;

    // scan-matcher using climb-hill 
    GMapping::OrientedPoint MRMC_ClimbHill(CNodeScan*, float& );
    GMapping::OrientedPoint m_pose_odo_now; 
    GMapping::OrientedPoint m_pose_odo_pri;
    GMapping::OrientedPoint m_pose_last; 
    GMapping::OrientedPoint m_pose_curr;
    // GMapping::ScanMatcher m_matcher; 
    
    // grid map
    GMapping::ScanMatcherMap * m_gridmap;
    void getGridMap(GMapping::ScanMatcher& matcher, GMapping::ScanMatcherMap* gridmap);
    void getKeyNodeGridMap(GMapping::ScanMatcher& matcher, GMapping::ScanMatcherMap* gridmap);

    // ICP Pairwise
    // CICPWarpper* m_icp;
    bool ICP_PairWise(CNodeScan*, GMapping::OrientedPoint&, int n, GMapping::OrientedPoint&, MatchingResult& , float, bool loop = false);
    float ICP_PairWise(CNodeScan*, CNodeScan*, double*, double*);
    PARAM_SET_GET(double, icpGoodness, protected, public, public);
    double m_icpReplaceThreshold; 
    inline void angleT(double& a); // [-M_PI, M_PI)

    // G2O
public:
    bool addEdgeToG2O(MatchingResult& , bool large_edge, bool set_estimate, bool replace = false); 
    void fromOrientedPoint2SE2(GMapping::OrientedPoint& trans, g2o::EdgeSE2& ret);
    void saveG2O(const char* fname = 0);
    double optimizeGraph(int iter=-1);
    
    // iterative g2o 
    bool use_iterative_g2o;
    double iterativeOptGraph(int iter = -1);
    #ifndef ONLINE
        g2o::SparseOptimizer * optimizer_; 
        g2o::SparseOptimizer * getNewOptimizer();
    #else
        g2o::SparseOptimizerOnline * optimizer_;
    #endif 
    
    // Path 
    void savePath(const char* fname = 0);
    void savePathG2O(const char* fname=0);
    
    // reconstructMap
    void updatePath(); // update path from g2o
    void updateKeyPath(); // update path of keynode from g2o

    // Marginal Cov
    void calMarginalCov(int id);
    void balanceCov(int loop_id);

    // loop detecter 
    void calCurCovariance(double * cur_cov, double theta );
    void calCurCovOdo();
    bool loopByCov(CNodeScan*, CNodeScan*);
    bool isLoopInCov(MatchingResult&);
    std::vector<int> getPotentialLoopNodes(CNodeScan* );
    int skip_latest_nodes_; 
    int loop_detect_step_;
    float sigma_cov_;
    float loop_cov_scale_; // update cov corresponding to loop
    // double acc_cov_[6];   // accumulated covariance
    Eigen::Matrix3d acc_cov_;
    
    double pose_dis_thresh_; // whether current pose changed big enough for reconstruct map

protected:
    void setParticlePoses(GMapping::OrientedPoint&);
    int m_nMRMC; // number of particles to predicate motion using climb-hill 
    double m_minScore; // minScore for climb-hill
    vector<GMapping::OrientedPoint> m_nposes;
    // int m_count; // number of nodes   
};


extern void testOptimizer(const char* fname);

#endif
