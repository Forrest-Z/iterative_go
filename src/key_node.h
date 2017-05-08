#ifndef KEY_NODE_H
#define KEY_NODE_H

#include "node_scan.h"
#include <vector>
#include <scanmatcher/scanmatcher.h>
#include "point.h"

// extern GMapping::ScanMatcher& getMatcher();

class MatchingResult;

class CKeyNode : public CNodeScan
{
public:
    CKeyNode(CNodeScan* root, int k_id, GMapping::ScanMatcherMap* map = 0); 
    CKeyNode(GMapping::RangeReading& , int k_id, GMapping::ScanMatcherMap* map = 0);
    virtual ~CKeyNode();
    void initRoot(GMapping::ScanMatcherMap*); // initialize the root node
    virtual float matchNodeForLoop(CNodeScan*, MatchingResult&);
    virtual float matchNodePair(CNodeScan* , MatchingResult& mr); 
    virtual float matchNodeICP(CNodeScan*, GMapping::OrientedPoint, MatchingResult&);
    virtual bool addNode(CNodeScan*);
    void reconstructMap(); // 
    int k_id_; 

    void setPose(GMapping::OrientedPoint& );
    OrientedPoint2D pose_; 
    
    vector<CNodeScan*> nodes_; 
    vector<bool> b_register_; // identify whether to rematch this node with key node
    void resetRegister();

    // grid map
    GMapping::ScanMatcherMap * gridmap_;
    void resetGridMap();

    // IO 
    virtual bool saveScanXY(const char* f = 0, bool binary = true);

    // prepare ICP 
    double occ_thresh_;
    bool b_icp_ready_;
    void prepareICP();
    vector<float> map_x_; 
    vector<float> map_y_;
    void setActive(); // icp not ready
    double cov_x_max_;  // max value for x,y covariance
    double cov_y_max_; 
    double loop_cov_scale_; // scale loop information

    // ClimbHill Matcher
    double minimumScore_; 

    // observation range to detected loop 
    float xmin_; 
    float xmax_; 
    float ymin_; 
    float ymax_;

    // to detect loop 
    double CH_step_; // Climb Hill step
    int getCloestPose(GMapping::OrientedPoint* , double& m_dis);
    // vector<GMapping::OrientedPoint> interporlatePose(GMapping::OrientedPoint sp, GMapping::OrientedPoint ep, int n);

};

#endif
