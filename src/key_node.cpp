#include "key_node.h"
#include "MatchingResult.h"
#include "ZHIcp_Warpper.h"
#include "debug.h"
#include "global_guys.h"

CKeyNode::CKeyNode(CNodeScan* root, int k_id, GMapping::ScanMatcherMap* map) : 
k_id_(k_id),
b_icp_ready_(false),
occ_thresh_(0.25), // default
minimumScore_(10), // ClimbHill Matcher
cov_x_max_(1.),
cov_y_max_(1.),
loop_cov_scale_(1.),
gridmap_(0),
CH_step_(0.05)
{
    // actually todo copy function
    m_id = root->m_id; 
    m_pose = new GMapping::OrientedPoint(*(root->getPose()));
    m_reading = root->m_reading; 
    m_x = root->m_x; 
    m_y = root->m_y; 
    m_timestamp = root->m_timestamp; 
    initRoot(map);
}

CKeyNode::CKeyNode(GMapping::RangeReading& r, int k_id,  GMapping::ScanMatcherMap* map):
CNodeScan(r), 
k_id_(k_id),
b_icp_ready_(false),
occ_thresh_(0.25) // default
{
    initRoot(map);
}

 void CKeyNode::resetGridMap()
{
    if(gridmap_ != 0)
    {
        delete gridmap_;
    }
    // default value 
    double xmin , ymin, xmax, ymax, delta; 
    xmin = ymin = -30.; 
    xmax = ymax = 30. ; 
    delta = 0.05; 
    gridmap_ = new GMapping::ScanMatcherMap(GMapping::Point(xmin+xmax, ymin+ymax)*.5, xmax - xmin, ymax - ymin, delta);
    return ;
}

void CKeyNode::reconstructMap()
{
    resetGridMap();
    for(int i=0; i<nodes_.size(); i++)
    {
        addMapFromNode(nodes_[i], gridmap_, getMatcher());
    }
}

void CKeyNode::setPose(GMapping::OrientedPoint& p)
{
    *m_pose = p; 
    pose_ = OrientedPoint2D(m_pose->x, m_pose->y, m_pose->theta);
}

void CKeyNode::resetRegister()
{
    for(int i=1; i<b_register_.size(); i++)
        b_register_[i] = false;
}

void CKeyNode::initRoot(GMapping::ScanMatcherMap* map)
{
    if(map == 0)
    {
        resetGridMap();
    }else
    {
        gridmap_ = new GMapping::ScanMatcherMap(*map);
    }
    // root pose 
    pose_ = OrientedPoint2D(m_pose->x, m_pose->y, m_pose->theta);
    // add self
    CNodeScan* pRoot = dynamic_cast<CNodeScan*>(this); 
    nodes_.push_back(pRoot);
    addMapFromNode(pRoot, gridmap_, getMatcher());
    b_register_.push_back(true);
}

CKeyNode::~CKeyNode()
{
    // i=0 is self, so not delete explicitly 
    for(int i=1; i<nodes_.size(); i++)
        delete nodes_[i];
    if(gridmap_) delete gridmap_;
}

bool CKeyNode::saveScanXY(const char* fname, bool binary)
{
    prepareICP();
    bool ret;
    if(binary)
    {
        ret = saveScanXY_bin_impl(map_x_, map_y_, fname);
    }else
    {
        ret = saveScanXY_impl(map_x_, map_y_, fname); 
    }
    return ret;
}

float CKeyNode::matchNodeForLoop(CNodeScan* new_node, MatchingResult& mr)
{
    GMapping::OrientedPoint cur_gp = *(new_node->getPose());
    GMapping::OrientedPoint corrected_gp;
    int n ; 
    GMapping::OrientedPoint dis_p = ominus(*m_pose, cur_gp); 
    n = sqrt(SQ(dis_p.x) + SQ(dis_p.y))/CH_step_; 
    ++n;
    vector<GMapping::OrientedPoint> potential_pose = interporlatePose(cur_gp, *m_pose, n);
    double score ;
    double bestScore = 0; 
    GMapping::OrientedPoint best_p;
    MatchingResult mr2; 
  
    double MC_score;
    double MC_bestScore = 0;
    // for debug
    // ofstream outf("keymatch.log");

    for(int i=0; i<n; i++)
    {
        // double score = ClimbHillMatcher(cur_gp, corrected_gp, gridmap_, &(new_node->m_reading[0]));
        MC_score = ClimbHillMatcher(potential_pose[i], corrected_gp, gridmap_, &(new_node->m_reading[0]));
        score = matchNodeICP(new_node, corrected_gp, mr2);
        
        // GMapping::OrientedPoint pose = ominus(cur_gp, corrected_gp);
        // outf<<k_id_<<" "<<new_node->m_id<<" "<<MC_score<<" "<<score<<endl;
        // outf<<&pose<<endl;
        if( score >= bestScore && MC_score > MC_bestScore)
        {
            bestScore = score; 
            best_p = corrected_gp;
            MC_bestScore = MC_score;
        }
    }
    /*if(score < minimumScore_)
    {
        return 0; 
    }*/
    OrientedPoint2D cor_gp(best_p.x, best_p.y, best_p.theta);
    OrientedPoint2D output = pose_.ominus(cor_gp);
    g2o::SE2 tmpSE(output.x, output.y, output.theta);
    mr.m_edge.setMeasurement(tmpSE);
    // cout<<"key_node.cpp: trans: "<<output.x<<" "<<output.y<<" score: "<<bestScore<<endl;
    // cout<<"key_node.cpp: best Score = "<<bestScore<<endl;
    return matchNodeICP(new_node, best_p, mr);
}

float CKeyNode::matchNodePair(CNodeScan* new_node, MatchingResult& mr)
{
    GMapping::OrientedPoint cur_gp = *(new_node->getPose());
    GMapping::OrientedPoint corrected_gp;
    /*int n ; 
    GMapping::OrientedPoint dis_p = ominus(*m_pose, cur_gp); 
    n = sqrt(SQ(dis_p.x) + SQ(dis_p.y))/CH_step_; 
    ++n;
    vector<GMapping::OrientedPoint> potential_pose = interporlatePose(cur_gp, *m_pose, n);
    double score ;
    double bestScore = 0; 
    GMapping::OrientedPoint best_p;
    for(int i=0; i<n; i++)
    {
        // double score = ClimbHillMatcher(cur_gp, corrected_gp, gridmap_, &(new_node->m_reading[0]));
        score = ClimbHillMatcher(potential_pose[i], corrected_gp, gridmap_, &(new_node->m_reading[0]));
        if(score > bestScore)
        {
            bestScore = score; 
            best_p = corrected_gp;
        }
    }*/
    double score = ClimbHillMatcher(cur_gp, corrected_gp, gridmap_, &(new_node->m_reading[0]));
    if(score < minimumScore_)
    {
        return 0; 
    }
    // ICP to get goodness
    OrientedPoint2D cor_gp(corrected_gp.x, corrected_gp.y, corrected_gp.theta);
    // OrientedPoint2D cor_gp(best_p.x, best_p.y, best_p.theta);
    OrientedPoint2D output = pose_.ominus(cor_gp);
    g2o::SE2 tmpSE(output.x, output.y, output.theta);
    mr.m_edge.setMeasurement(tmpSE);
    return matchNodeICP(new_node, corrected_gp, mr);
}

float CKeyNode::matchNodeICP(CNodeScan* new_node, GMapping::OrientedPoint iniPose, MatchingResult& mr)
{
    prepareICP();
    OrientedPoint2D cur_pose = OrientedPoint2D(iniPose.x, iniPose.y, iniPose.theta);
    OrientedPoint2D rel_pose = pose_.ominus(cur_pose); // cur_pose.ominus(pose_); 
    double input[3] = {rel_pose.x, rel_pose.y, rel_pose.theta};
    double output[3] = {0};
    float goodness  = getICPMatcher()->ICPMatch(&map_x_[0], &map_y_[0], map_x_.size(), 
                            &(new_node->m_x[0]),&(new_node->m_y[0]), CNodeScan::N_BEAMS, input, output);
 
    // covariance to detect validity
    double p[3]; 
    double cov[6]; 
    getICPMatcher()->getResult(p, cov);
    Eigen::Matrix3d covM = Eigen::Matrix3d::Identity();
    covM(0,0) = cov[0]; 
    covM(1,1) = cov[3]; 
    covM(2,2) = cov[5];
    Eigen::Matrix3d invM = covM.inverse()*loop_cov_scale_;
    // 
    if(covM(0,0) > cov_x_max_ || covM(1,1) > cov_y_max_)
    {
        cout<<"key_node.cpp: cov_x: "<<covM(0,0)<<" cov_y: "<<covM(1,1)<<" exceed valid cov!"<<endl;
        return 0; 
    }
    // 
    mr.id1 = m_id;
    mr.id2 = new_node->m_id;
    mr.m_edge.setInformation(invM);
    return goodness;
}

int CKeyNode::getCloestPose(GMapping::OrientedPoint* pquery, double& m_dis)
{
    double min_dis = 1000000;
    double t_dis;
    int ret= -1;
    for(int i=0; i<nodes_.size(); i++)
    {
        CNodeScan* pt = nodes_[i]; 
        GMapping::OrientedPoint* p = pt->getPose(); 
        GMapping::OrientedPoint e = ominus(*p, *pquery); 
        t_dis = SQ(e.x) + SQ(e.y);
        if( t_dis < min_dis)
        {
            min_dis = t_dis; 
            ret = pt->m_id;
        }
    }
    m_dis = min_dis;
    return ret;
}

/*
float CKeyNode::matchNodePair(CNodeScan* new_node, MatchingResult& mr)
{
    prepareICP();
    GMapping::OrientedPoint cur_gp = *(new_node->getPose());
    OrientedPoint2D cur_pose = OrientedPoint2D(cur_gp.x, cur_gp.y, cur_gp.theta);
    OrientedPoint2D rel_pose = pose_.ominus(cur_pose); // cur_pose.ominus(pose_); 
    double input[3] = {rel_pose.x, rel_pose.y, rel_pose.theta};
    double output[3] = {0};
    float goodness  = getICPMatcher()->ICPMatch(&map_x_[0], &map_y_[0], map_x_.size(), 
                            &(new_node->m_x[0]),&(new_node->m_y[0]), CNodeScan::N_BEAMS, input, output);
                            
    if(goodness > 0.95)
    {
        // potential loop closure 
        *//*mr.id1 = m_id;
        mr.id2 = new_node->m_id;
        OrientedPoint transPose = ominus(last_pose, output);
        angleT(transPose.theta);
        fromOrientedPoint2SE2(transPose, mr.m_edge);
        // mr.m_edge.setInformation(Eigen::Matrix3d::Identity());
        mr.m_edge.setInformation(covM);*/
     //   cout<<"key_node.cpp: potential loop match between new_node: "<<new_node->m_id<<" with key_node: "<<k_id_<<endl;
    /*}
    double p[3]; 
    double cov[6]; 
    getICPMatcher()->getResult(p, cov);
    Eigen::Matrix3d covM = Eigen::Matrix3d::Identity();
    covM(0,0) = cov[0]; 
    covM(1,1) = cov[3]; 
    covM(2,2) = cov[5];
    Eigen::Matrix3d invM = covM.inverse()*0.02;
    // cout<<"key_node.cpp: key_node cov: "<<covM(0,0)<<" "<<covM(1,1)<<" "<<covM(2,2)<<endl;
    // cout<<"key_node.cpp: key_node inf: "<<invM(0,0)<<" "<<invM(1,1)<<" "<<invM(2,2)<<endl;

    mr.id1 = m_id;
    mr.id2 = new_node->m_id;
    g2o::SE2 tmpSE(output[0], output[1], output[2]);
    mr.m_edge.setMeasurement(tmpSE);
    // fromOrientedPoint2SE2(transPose, mr.m_edge);
    // mr.m_edge.setInformation(Eigen::Matrix3d::Identity());
    mr.m_edge.setInformation(invM);
    return goodness;
}
*/
bool CKeyNode::addNode(CNodeScan* new_node)
{
    addMapFromNode(new_node, gridmap_, getMatcher());
    nodes_.push_back(new_node);
    b_register_.push_back(false);
    return true;
}

void CKeyNode::setActive()
{
    b_icp_ready_ = false;
}

void CKeyNode::prepareICP()
{
    if(b_icp_ready_)
    {
        // ALREADY OK 
        return ;
    }
    map_x_.clear();
    map_y_.clear();
    for(int x=0; x < gridmap_->getMapSizeX(); x++)
    {
        for(int y=0; y < gridmap_->getMapSizeY(); y++)
        {   
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            GMapping::IntPoint p(x, y); 
            GMapping::PointAccumulator& pt = gridmap_->cell(p); 
            double occ = pt;
            assert(occ <= 1.0);
            if(occ > occ_thresh_)
            {
                GMapping::Point pp = pt.mean();
                Point2D tmp(pp.x, pp.y); 
                Point2D trans = pose_.ominus(tmp); // transfrom to robot coordinator
                map_x_.push_back(trans.x);
                map_y_.push_back(trans.y);
                // map_x_.push_back(pp.x);
                // map_y_.push_back(pp.y);
            }
            /*
            if(occ < 0)
                m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = -1; 
            else if(occ > occ_thresh_)
            {
                //m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = (int)round(occ*100.0);
                // m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = 100;
            }
            else
                m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = 0;
            */
        }
    }
    b_icp_ready_ = true;
}
