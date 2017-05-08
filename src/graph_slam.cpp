#include "graph_slam.h"
#include "debug.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "ZHIcp_Warpper.h"
// #include "ZHCanonical_Matcher.h"
#include "point.h"

// #include "g2o/math_groups/se2.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

// #include "g2o/core/graph_optimizer_sparse.h"
// #include "g2o/core/hyper_dijkstra.h" 

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/core/sparse_block_matrix.h"
#include "MatchingResult.h"

#include "ros/ros.h"

// #define DEBUG 1

using namespace GMapping;
#define SQ(x) ((x)*(x))
#ifndef R2D
#define R2D(x) (((x)*180.)/M_PI)
#endif 

#ifndef D2R
#define D2R(x) (((x)*M_PI)/180.)
#endif

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1,-1> > SlamBlockSolver; 
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver; 
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver; 

typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*> VertexIDMap;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

CGraph_SLAM::CGraph_SLAM(double minScore, int nParticles): 
m_nMRMC(nParticles),
m_minScore(minScore),
m_gridmap(0), 
// m_icp(new CICPWarpper),
optimizer_(0),
skip_latest_nodes_(10), 
loop_detect_step_(1),
sigma_cov_(2),
loop_cov_scale_(5),
m_icpReplaceThreshold(0.9), // 0.9
current_key_node_(0),
sqr_dis_thresh_(9.), // 5m to start a new keynode
m_cov_motion_scale(1.737), 
m_cov_angle_scale(3),
m_extra_samples(5),
m_score_scale(0.7)
{
    assert(m_nMRMC > 0);
    m_nposes.resize(m_nMRMC);
    
    ros::NodeHandle nh("~"); 
    nh.param("rebuild_map_thresh", pose_dis_thresh_, 0.1); 
    nh.param("laser_odometry", use_laser_odo_, false);
    nh.param("loop_correspondence", m_icpReplaceThreshold, 0.9);
    nh.param("lstep", m_CH_step, 0.05);
    nh.param("iterative_g2o",use_iterative_g2o, false);
    if(!nh.getParam("knode_switch_sqr_dis", sqr_dis_thresh_))
        sqr_dis_thresh_ = 25; // 5m 
    if(!nh.getParam("extra_samples", m_extra_samples))
        m_extra_samples = 5; // extra samples for the pose interpolation
    // m_icp->initICP();
#ifndef ONLINE
    /* old g2o config
    optimizer_ = new g2o::SparseOptimizer();
    SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
    linearSolver->setBlockOrdering(false); 
    SlamBlockSolver * solver = new SlamBlockSolver(optimizer_, linearSolver); 
    optimizer_->setSolver(solver);
    */
    // new g2o config 
    optimizer_ = getNewOptimizer();// new g2o::SparseOptimizer(); 
}

g2o::SparseOptimizer * CGraph_SLAM::getNewOptimizer()
{
    g2o::SparseOptimizer * optimizer = new g2o::SparseOptimizer(); 
    SlamLinearCholmodSolver * linearSolver = new SlamLinearCholmodSolver(); 
    // SlamLinearCSparseSolver * linearSolver = new SlamLinearCSparseSolver();
    linearSolver->setBlockOrdering(false); 
    SlamBlockSolver * blockSolver = new SlamBlockSolver(linearSolver); 
    g2o::OptimizationAlgorithmLevenberg* solverLevenberg = new g2o::OptimizationAlgorithmLevenberg(blockSolver); 
    g2o::OptimizationAlgorithmGaussNewton* solverGauss = new  g2o:: OptimizationAlgorithmGaussNewton(blockSolver);
    optimizer->setAlgorithm(solverGauss);
#else
    optimizer = new g2o::SparseOptimizerIncremental();
    optimizer->initSolver(3.);
#endif
    optimizer->setVerbose(true);
    return optimizer;

}

CGraph_SLAM::~CGraph_SLAM()
{
    if(m_gridmap) delete m_gridmap;
    // if(m_icp) delete m_icp;
    if(optimizer_) delete optimizer_;
    // if(current_key_node_) delete current_key_node_;
    // TODO:: delete graphs 
    iterator it = begin();
    while(it!=end())
    {
        delete it->second; 
        ++it; 
    }
    key_iterator kit = kbegin();
    while(kit!=kend())
    {
        delete kit->second; 
        ++kit; 
    }
}


void CGraph_SLAM::setParticlePoses(GMapping::OrientedPoint& p)
{
    m_nposes.resize(m_nMRMC);
    for(int i=0; i<m_nposes.size(); i++)
    {
        m_nposes[i] = p; 
    }
}

GMapping::OrientedPoint CGraph_SLAM::MRMC_ClimbHill(CNodeScan* new_node, float& ret_score)
{
    if(m_graph.size()<=0) 
    {
        cout<<"graph_slam.cpp: map not be intialized!"<<endl;
        return GMapping::OrientedPoint();
    }
    double tx, ty, ttheta, ts;
    tx = ty = ttheta = ts = 0;
    double best_tx, best_ty, best_ttheta;
    double maxScore = 0 ; // m_minScore;
    int k= 0;
    // i== 0 keep the odometry info
    for(int i=0; i<m_nposes.size(); i++)
    {
        OrientedPoint corrected; 
          
        //double score = getMatcher().optimize(corrected, *m_gridmap, m_nposes[i], &(new_node->m_reading[0]));
        double score = ClimbHillMatcher(m_nposes[i], corrected, m_gridmap, &(new_node->m_reading[0]));
        // cout<<"graph_slam.cpp: after optimize! score = "<<score<<endl;

        if(score > maxScore)
        {
            best_tx = corrected.x;
            best_ty = corrected.y;
            best_ttheta = corrected.theta;
            maxScore = score;
        }

        if(score < m_minScore)
        {
           continue;
           // score = m_minScore; 
           // corrected = *(new_node->getPose());
        }else
        {
            tx += corrected.x * score; 
            ty += corrected.y * score; 
            ttheta += corrected.theta * score;  
            ts += score;
            k++;
        }       
    }
    if(ts > 0)
    {
       cout<<"graph_slam.cpp: average score for MCMC is: "<< (ts/k) <<" with "<<k<<" samples"<<endl;
       ret_score = (ts/k);
       return OrientedPoint(tx/ts, ty/ts, ttheta/ts);
    }
    OrientedPoint iniPose(best_tx, best_ty, best_ttheta);
    ret_score = maxScore;
    cout<<"graph_slam.cpp: best score is : "<<maxScore<<endl;
    return iniPose;
}

float CGraph_SLAM::ICP_PairWise(CNodeScan* prev_node, CNodeScan* new_node, double* input, double * output)
{
    int num =  CNodeScan::N_BEAMS;
    float goodness = getICPMatcher()->ICPMatch(&prev_node->m_x[0], &prev_node->m_y[0], 
            &new_node->m_x[0], &new_node->m_y[0], num ,
            input, output); 
    return goodness;
}

void CGraph_SLAM::calCurCovOdo()
{
    acc_cov_(0,0) += m_motionModel.stt; 
    acc_cov_(1,1) += m_motionModel.stt; 
    acc_cov_(2,2) += m_motionModel.srr;
}

// [-M_PI, M_PI)
inline void CGraph_SLAM::angleT(double& a)
{
    while(a>=M_PI)
    {
        a -= 2*M_PI;
    }
    while(a<-M_PI)
    {
        a += 2*M_PI; // bug : This is 2*M_PI not M_PI, detected at 2014.3.18 David Z
    }
}
#ifdef DEBUG
namespace
{
    void recordScan(ofstream& ouf2, CNodeScan* pn)
    {
        for(int i=0; i<pn->m_reading.size(); i++)
        {
            ouf2<<pn->m_reading[i]<<" ";
        }
        ouf2<<endl;
    }
}
#endif


/*
bool CGraph_SLAM::ICP_PairWise(CNodeScan* new_node, GMapping::OrientedPoint& iniPose, int n, 
                                GMapping::OrientedPoint& output, MatchingResult& mr, bool loop)
{
    bool ret ;
    assert(n < m_graph.size());
    CNodeScan* prev_node = m_graph[n]; 
    GMapping::OrientedPoint last_pose = *(prev_node->getPose());
    OrientedPoint2D last_pose_2d( last_pose.x, last_pose.y, last_pose.theta); 
    OrientedPoint2D curr_pose_2d( iniPose.x, iniPose.y , iniPose.theta);
    OrientedPoint2D rel_ini_2d = last_pose_2d.ominus(curr_pose_2d);
    angleT(rel_ini_2d.theta);

    double input_p[3] = {rel_ini_2d.x, rel_ini_2d.y, rel_ini_2d.theta}; 
    double output_p[3] = {0}; 
    float goodness = ICP_PairWise(prev_node, new_node, input_p, output_p);

    Eigen::Matrix3d covM = Eigen::Matrix3d::Identity();
    if(goodness > m_icpGoodness)
    {
        if(goodness > m_icpReplaceThreshold || loop)
        {
            if(loop)
            {
                // noisy loop closure
                if( ( SQ(output_p[0]) + SQ(output_p[1])) > sigma_cov_*(fabs(acc_cov_(0,0)) + fabs(acc_cov_(1,1))) || SQ(output_p[2]) > sigma_cov_ * acc_cov_(2,2)) 
                {
                    return false;
                }
            }
            cout<<"graph_slam.cpp: replace with ICP goodness: "<<goodness<<endl; 
            OrientedPoint2D rel_icp(output_p[0], output_p[1], output_p[2]); 
            curr_pose_2d = last_pose_2d.oplus(rel_icp);
            angleT(curr_pose_2d.theta);
            output = GMapping::OrientedPoint(curr_pose_2d.x, curr_pose_2d.y, curr_pose_2d.theta); 
#ifdef DEBUG
    {
        static ofstream ouf("icp_result.txt");
        static ofstream prev_f("icp_scan_prev.txt");
        static ofstream curr_f("icp_scan_curr.txt");
       
        recordScan(prev_f, prev_node); 
        recordScan(curr_f, new_node);
        ouf<<input_p[0]<<" "<<input_p[1]<<" "<<input_p[2]<<" "<<output_p[0]<<" "<<output_p[1]<<" "<<output_p[2]<<" "<<goodness<<endl;
        // ouf<<iniPose.x<<" "<<iniPose.y<<" "<<iniPose.theta<<" "<<curr_pose_2d.x<<" "<<curr_pose_2d.y<<" "<<curr_pose_2d.theta<<endl;
        // ouf<<iniPose.x<<" "<<iniPose.y<<" "<<iniPose.theta<<" "<<rel_icp.x<<" "<<rel_icp.y<<" "<<rel_icp.theta<<" "<<goodness<<endl;
    }
#endif
        }else
        {
            output = iniPose;
        }
        // calculate covariance according to icp result
        double p[3]; 
        double cov[6];
        getICPMatcher()->getResult(p, cov);
        covM(0,0) = cov[0] ; 
        covM(1,1) = cov[3]  ;
        covM(2,2) = cov[5]  ;
        
        if(!loop)
        {
            calCurCovariance(cov, p[2]);
            //cout<<"graph_slam.cpp: after calCurCovariance()"<<endl;
            // cout<<"graph_slam.cpp: after icp cov_x: "<<acc_cov_(0,0)<<" cov_y: "<< acc_cov_(1,1)<<endl;
        }else
        {
            // TODO:
            // merge covariance according to Tipaldi 
            // < Approximate Covariance Estimation in Graphical Approaches to SLAM >
            covM *= loop_cov_scale_;    
        }
        ret = true;
    }else
    {
        output = iniPose;
        if(!loop)
        {
            // cout<<"graph_slam.cpp: poor icp goodness: "<<goodness<<endl;
            calCurCovOdo();
            covM(0,0) = covM(1,1) = m_motionModel.stt;
            covM(2,2) = m_motionModel.srr;
            // cout<<"graph_slam.cpp: icp failed! cov_x: "<<acc_cov_(0,0)<<" cov_y: "<< acc_cov_(1,1)<<endl;
        }
        ret = false;
    }
    
    mr.id1 = prev_node->m_id; 
    mr.id2 = new_node->m_id; 
    OrientedPoint transPose = ominus(last_pose, output);
    angleT(transPose.theta);
    fromOrientedPoint2SE2(transPose, mr.m_edge);
    // mr.m_edge.setInformation(Eigen::Matrix3d::Identity());
    mr.m_edge.setInformation(covM);
    // cout<<"graph_slam.cpp: after set matchingResult!"<<endl;
    return ret;
}*/

bool CGraph_SLAM::isLoopInCov(MatchingResult& mr)
{
    GMapping::OrientedPoint* p1 = m_graph[mr.id1]->getPose();
    GMapping::OrientedPoint* p2 = m_graph[mr.id2]->getPose(); 
    // Eigen::Vector3d t = mr.m_edge.toVector(); 
    double t[3];
    mr.m_edge.getMeasurementData(t);
    OrientedPoint2D trans(t[0], t[1], t[2]); 
    OrientedPoint2D tp1(p1->x, p1->y, p1->theta); 
    OrientedPoint2D tp2 = tp1.oplus(trans);
    double abs_dis = sqrt(SQ(tp2.x - p2->x) + SQ(tp2.y - p2->y));
    if( abs_dis > sigma_cov_*(fabs(acc_cov_(0,0)) + fabs(acc_cov_(1,1))))    
    {
        ROS_WARN("graph_slam.cpp: misleading loop cov_x: %f, cov_y: %f", acc_cov_(0,0), acc_cov_(1,1));
        return false;
    }
    double dis_angle = tp2.theta - p2->theta;
    angleT(dis_angle);
    if(fabs(dis_angle) > sigma_cov_ * fabs(acc_cov_(2,2)))
    {
        ROS_WARN("graph_slam.cpp: misleading loop cov_theta %f", acc_cov_(2,2));
        return false;
    }
    return true;
}

float CGraph_SLAM::scoreWeight(float s)
{
    if(s < m_minScore)
    {
        return ((m_minScore - s)/m_minScore);
    }
    return 0;
}

bool CGraph_SLAM::ICP_PairWise(CNodeScan* new_node, GMapping::OrientedPoint& iniPose, int n, 
                                GMapping::OrientedPoint& output, MatchingResult& mr, float score, 
                                bool loop)
{
    bool ret ;
    assert(n < m_graph.size());
    CNodeScan* prev_node = m_graph[n]; 
    GMapping::OrientedPoint last_pose = *(prev_node->getPose());
    OrientedPoint2D last_pose_2d( last_pose.x, last_pose.y, last_pose.theta); 
    OrientedPoint2D curr_pose_2d( iniPose.x, iniPose.y , iniPose.theta);
    OrientedPoint2D rel_ini_2d = last_pose_2d.ominus(curr_pose_2d);
    angleT(rel_ini_2d.theta);

    double input_p[3] = {rel_ini_2d.x, rel_ini_2d.y, rel_ini_2d.theta}; 
    double output_p[3] = {0}; 
    float goodness = ICP_PairWise(prev_node, new_node, input_p, output_p);

    Eigen::Matrix3d covM = Eigen::Matrix3d::Identity();
    if(goodness > m_icpGoodness)
    {
        if(goodness > m_icpReplaceThreshold || loop)
        {
            if(loop)
            {
                // noisy loop closure
                if( ( SQ(output_p[0]) + SQ(output_p[1])) > sigma_cov_*(fabs(acc_cov_(0,0)) + fabs(acc_cov_(1,1))) || SQ(output_p[2]) > sigma_cov_ * acc_cov_(2,2)) 
                {
                    return false;
                }
            }
            // 2014.3.28 ICP result in accumulated errror, 
            // cout<<"graph_slam.cpp: replace with ICP goodness: "<<goodness<<endl; 
            // OrientedPoint2D rel_icp(output_p[0], output_p[1], output_p[2]); 
            // curr_pose_2d = last_pose_2d.oplus(rel_icp);
            // angleT(curr_pose_2d.theta);
            output = iniPose; // GMapping::OrientedPoint(curr_pose_2d.x, curr_pose_2d.y, curr_pose_2d.theta); 
#ifdef DEBUG
    {
        static ofstream ouf("icp_result.txt");
        static ofstream prev_f("icp_scan_prev.txt");
        static ofstream curr_f("icp_scan_curr.txt");
       
        recordScan(prev_f, prev_node); 
        recordScan(curr_f, new_node);
        ouf<<input_p[0]<<" "<<input_p[1]<<" "<<input_p[2]<<" "<<output_p[0]<<" "<<output_p[1]<<" "<<output_p[2]<<" "<<goodness<<endl;
        // ouf<<iniPose.x<<" "<<iniPose.y<<" "<<iniPose.theta<<" "<<curr_pose_2d.x<<" "<<curr_pose_2d.y<<" "<<curr_pose_2d.theta<<endl;
        // ouf<<iniPose.x<<" "<<iniPose.y<<" "<<iniPose.theta<<" "<<rel_icp.x<<" "<<rel_icp.y<<" "<<rel_icp.theta<<" "<<goodness<<endl;
    }
#endif
        }else
        {
            output = iniPose;
        }
        // calculate covariance according to icp result
        double p[3]; 
        double cov[6];
        getICPMatcher()->getResult(p, cov);
        cov[0] += m_motionModel.stt * scoreWeight(score);
        cov[3] += m_motionModel.stt * scoreWeight(score);
        cov[5] += m_motionModel.srr * scoreWeight(score);
        
        // ignore non-dignal elements
        cov[1] = cov[2] = cov[4] = 0;

        covM(0,0) = cov[0]; //  + m_motionModel.stt * scoreWeight(score); 
        covM(1,1) = cov[3]; //  + m_motionModel.stt * scoreWeight(score);
        covM(2,2) = cov[5]; //  + m_motionModel.srr * scoreWeight(score);
        
        if(!loop)
        {
            calCurCovariance(cov, p[2]);
            //cout<<"graph_slam.cpp: after calCurCovariance()"<<endl;
            // cout<<"graph_slam.cpp: after icp cov_x: "<<acc_cov_(0,0)<<" cov_y: "<< acc_cov_(1,1)<<endl;
        }else
        {
            // TODO:
            // merge covariance according to Tipaldi 
            // < Approximate Covariance Estimation in Graphical Approaches to SLAM >
            covM *= loop_cov_scale_;    
        }
        ret = true;
    }else
    {
        output = iniPose;
        if(!loop)
        {
            // cout<<"graph_slam.cpp: poor icp goodness: "<<goodness<<endl;
            calCurCovOdo();
            covM(0,0) = covM(1,1) = m_motionModel.stt;
            covM(2,2) = m_motionModel.srr;
            // cout<<"graph_slam.cpp: icp failed! cov_x: "<<acc_cov_(0,0)<<" cov_y: "<< acc_cov_(1,1)<<endl;
        }
        ret = false;
    }
    if(!loop)
    {
        mr.id1 = 0; 
        fromOrientedPoint2SE2(output, mr.m_edge);
        // covM(0,0) = acc_cov_(0,0); 
        // covM(1,1) = acc_cov_(1,1);
        // covM(2,2) = acc_cov_(2,2);
        // Eigen::Matrix3d invM = covM.inverse();
        // cout<<"graph_slam.cpp: cur acc_cov: "<<covM(0,0)<<" "<<covM(1,1)<<" "<<covM(2,2)<<endl;
        // cout<<"graph_slam.cpp: cur acc_inf: "<<invM(0,0)<<" "<<invM(1,1)<<" "<<invM(2,2)<<endl;
    }else
    {
        mr.id1 = prev_node->m_id;   
        OrientedPoint transPose = ominus(last_pose, output);
        angleT(transPose.theta);
        fromOrientedPoint2SE2(transPose, mr.m_edge);
    }
    mr.id1 = prev_node->m_id; 
    mr.id2 = new_node->m_id; 
    OrientedPoint transPose = ominus(last_pose, output);
    angleT(transPose.theta);
    fromOrientedPoint2SE2(transPose, mr.m_edge);
 
    // mr.id2 = new_node->m_id; 
    // mr.m_edge.setInformation(Eigen::Matrix3d::Identity());
    mr.m_edge.setInformation(covM.inverse());
    // cout<<"graph_slam.cpp: after set matchingResult!"<<endl;
    return ret;
}

void CGraph_SLAM::savePathG2O(const char* f)
{
    fstream ouf;
    if(f == 0)
    {
        ouf.open("graph_path_g2o.txt", std::ios_base::out);
    }else
    {
        ouf.open(f, std::ios_base::out); 
    }
    iterator it = begin();
    int cnt = 0;
    while(it != end())
    {
        g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(it->first));
        assert(it->first == it->second->m_id && "node id not equal");
        double translation[3];
        pVertex->getEstimateData(translation);
        ouf<<std::fixed<<it->second->m_timestamp<<" "<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<std::endl;        
        // ouf<<std::fixed<<pn->m_timestamp<<" "<<pose->x<<" "<<pose->y<<" "<<pose->theta<<endl;
        ++it;  
    }
    cout<<"graph_slam.cpp: succeed to save Path!"<<endl;
}

void CGraph_SLAM::savePath(const char* f)
{
    fstream ouf; 
    if(f == 0)
    {
        ouf.open("graph_path.txt", std::ios_base::out);
    }else
    {
        ouf.open(f, std::ios_base::out); 
    }
    iterator it = begin();
    int cnt = 0;
    while(it != end())
    {
        CNodeScan* pn = it->second;
        GMapping::OrientedPoint* pose = pn->getPose();
        ouf<<std::fixed<<pn->m_timestamp<<" "<<pose->x<<" "<<pose->y<<" "<<pose->theta<<endl;
        ++it;  
    }
    cout<<"graph_slam.cpp: succeed to save Path!"<<endl;
}

void CGraph_SLAM::saveG2O(const char* f)
{
    if(f == 0)
    {
        optimizer_->save("g2o.log");
    }else
    {
        optimizer_->save(f); 
    }
    cout<<"graph_slam.cpp: succeed to save g2o!"<<endl;
}

void CGraph_SLAM::fromOrientedPoint2SE2(GMapping::OrientedPoint& trans, g2o::EdgeSE2& ret)
{
    g2o::SE2 tmpSE(trans.x,trans.y,trans.theta);
    ret.setMeasurement(tmpSE);
    // ret.setInverseMeasurement(tmpSE.inverse());
    return ;
}

void CGraph_SLAM::calCurCovariance(double * cur_cov, double theta)
{
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity(); 
    cov(0,0) = cur_cov[0]; 
    cov(0,1) = cov(1,0) = cur_cov[1]; 
    cov(0,2) = cov(2,0) = cur_cov[2]; 
    cov(1,1) = cur_cov[3]; 
    cov(1,2) = cov(2,1) = cur_cov[4]; 
    cov(2,2) = cur_cov[5];
    
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(0,0) = R(1,1) = cosf(theta); 
    R(0,1) = -sinf(theta); 
    R(1,0) = -R(0,1); 

    Eigen::Matrix3d acc_cov = R.transpose()*acc_cov_*R;
    acc_cov(1,0) = acc_cov(0,1) = acc_cov(1,2) = acc_cov(2,1) = acc_cov(0,2) = acc_cov(2,0) = 0;
    acc_cov_ = acc_cov + cov;
}

bool CGraph_SLAM::loopByCov(CNodeScan* p1, CNodeScan* p2)
{
    OrientedPoint pt1 = *(p1->getPose());
    OrientedPoint pt2 = *(p2->getPose());
    OrientedPoint dis = pt1 - pt2;
    return (SQ(dis.x) <= sigma_cov_*acc_cov_(0,0))&&(SQ(dis.y) <= sigma_cov_*acc_cov_(1,1));
}

std::vector<int> CGraph_SLAM::getPotentialLoopKeyNodes(CNodeScan* new_node)
{
    vector<int> ret; 
    if(m_key_graph.size()<=0) 
    {
        return ret; 
    }
    key_iterator it = kbegin();
    double potential_loop_dis; 
    ros::NodeHandle nh("~");
    if(!nh.getParam("potential_loop_dis", potential_loop_dis))
        potential_loop_dis = 4;
    // potential_loop_dis = potential_loop_dis > sqr_dis_thresh_? sqr_dis_thresh_: potential_loop_dis;
    while(it!=kend())
    {
        if(it->first == current_key_node_->k_id_ || it->second->nodes_.size() < 5) 
        {
           ++it;
           continue; // skip current_key_node
        }
        if( sqrDisPose(new_node->getPose(), it->second->getPose()) <= sqr_dis_thresh_)
        {
            double dis; 
            int n_l = it->second->getCloestPose(new_node->getPose(), dis);
            if(dis <= potential_loop_dis)
                ret.push_back(it->first);
            else
            {
                ROS_WARN("graph_slam.cpp: Tl satisfy, but minimum dis: %f > loop_dis: %f", dis, potential_loop_dis);
            }
        }   
        ++it;
    }
    return ret;
}

std::vector<int> CGraph_SLAM::getPotentialLoopNodes(CNodeScan* new_node)
{
    vector<int> ret;
    if(m_graph.size() <= skip_latest_nodes_) 
        return ret;
    for(int i=0; i<m_graph.size() - skip_latest_nodes_; i+= loop_detect_step_)
    {
        CNodeScan* prev_node = m_graph[i];
        if(loopByCov(prev_node, new_node)) 
            ret.push_back(i);
    }
    return ret;
}

void CGraph_SLAM::updateKeyPath()
{
    // update key node 
    key_iterator kit = kbegin();
    while(kit!= kend())
    {
         g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(kit->second->m_id));
         double t[3]; 
         pVertex->getEstimateData(t); 
         GMapping::OrientedPoint tmp(t[0], t[1], t[2]);
         kit->second->setPose(tmp);
         ++kit; 
    }
}

void CGraph_SLAM::updatePath()
{
    // update normal node
    iterator it = begin();
    while(it!=end())
    {
        g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(it->first));
        double t[3]; 
        pVertex->getEstimateData(t); 
        GMapping::OrientedPoint tmp(t[0], t[1], t[2]);
        it->second->setPose(tmp);
        ++it;
    } 
}

void CGraph_SLAM::reconstructKeyMap()
{
    cout<<"graph_slam.cpp : start to reconstructKeyMap()"<<endl;
    key_iterator kit = kbegin();
    while(kit!=kend())
    {
        kit->second->reconstructMap();
        kit++;
    }
}

void CGraph_SLAM::rebuildMapAndG2O()
{
    cout<<"graph_slam.cpp: start to rebuild Map and G2O!"<<endl;
    GMapping::ScanMatcherMap * new_map = new GMapping::ScanMatcherMap(m_gridmap->getCenter(),                                        m_gridmap->getWorldSizeX(), m_gridmap->getWorldSizeY(), 
                             m_gridmap->getDelta());
    // add first node 
    iterator it = begin();
    addMapFromNode(it->second, new_map, getMatcher());
    m_gridmap = new_map;
    
    // reset acculated covariance 
    acc_cov_ = Eigen::Matrix3d::Zero();

    iterator it_prev = it;
    it++;
    g2o::VertexSE2* pVertex;
    double t[3]; 
    OrientedPoint iniPose; 
    OrientedPoint outPose;
    MatchingResult mr;

    GMapping::OrientedPoint pose_odo_pri, pose_odo_now; 
    // first pose 
    // m_pose_odo_pri = m_pose_odo_now = *(it_prev->second->getPose());
    // setParticlePoses(m_pose_odo_pri);
    pose_odo_pri = pose_odo_now = *(it_prev->second->getPose());
    setParticlePoses(pose_odo_pri);

    // used to reconstruct successive edges
    double input[3] = {0}; 
    double output[3] = {0};
    
    int cnt = 0;
    int total_cnt = m_graph.size();

    vector<GMapping::OrientedPoint> swap_pose;

    int replace_num = 0;
    while(it != end())
    {
     //   ROS_ERROR("graph_slam.cpp: rebuild successive edge process (%d/%d)", ++cnt, total_cnt);
        // set pose from G2O 
        g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(it->first));
        pVertex->getEstimateData(t); 
        GMapping::OrientedPoint tmp(t[0], t[1], t[2]);
        // it->second->setPose(tmp);
        
        // bad: 1 ICP to predict a possible initial pose 
        /*GMapping::OrientedPoint trans = ominus(m_pose_odo_pri, tmp);
        input[0] = trans.x; input[1] = trans.y; input[2] = trans.theta;
        CNodeScan* pre_node = it_prev->second;
        CNodeScan* new_node = it->second;

        float goodness = getICPMatcher()->ICPMatch(&pre_node->m_x[0], &pre_node->m_y[0], 
                                    pre_node->m_x.size(),&new_node->m_x[0], 
                                    &new_node->m_y[0], new_node->m_x.size(), input, output); 
        m_pose_odo_now = oplus(m_pose_odo_pri, OrientedPoint(output[0], output[1], output[2]));
        */
        // good: 1 interporlate to generate many possible poses
        pose_odo_now = tmp;

        // bad: 2 MC to generate many possible values 
        /*for(int i=0; i<m_nposes.size(); i++)
        {
            OrientedPoint& p = m_nposes[i];
            p = m_motionModel.drawFromMotion(m_nposes[i], m_pose_odo_now, m_pose_odo_pri); 
        }*/
        pose_odo_pri = *(it->second->getPose());
        
        GMapping::OrientedPoint dis_p = ominus(pose_odo_pri, pose_odo_now); 
        int int_p = sqrt(SQ(dis_p.x) + SQ(dis_p.y))/m_CH_step; 
        int_p += m_extra_samples; // extra number of samples
        m_nposes = interporlatePose(pose_odo_pri, pose_odo_now, int_p /*m_nposes.size()*/);
 
        swap_pose = checkPoseCov(m_nposes, it->first);
        swap_pose.swap(m_nposes);

        // 2 climb-hill to verify possible pose  
        float score;
        OrientedPoint iniPose = MRMC_ClimbHill(it->second, score);
        swap_pose.swap(m_nposes);
        // ClimbHillMatcher(tmp, iniPose, m_gridmap, &(it->second->m_reading[0]));
        
        // 3 score and covariance check
        if(exceedScore(it->first, score) || exceedCov(ominus(pose_odo_pri,iniPose), it->first))
        {
            iniPose = pose_odo_pri;
        }
        else
        {
            // ROS_ERROR("graph_slam.cpp: replace edge %d for total: %d edge", ++replace_num, total_cnt);
        }
        // 4 use icp to generate transformation info
        ICP_PairWise(it->second, iniPose, it_prev->first, outPose, mr, score);
        
        if(!exceedScore(it->first, score) && !m_loop_node[it->first])
        {
            // 5 replace g2o info  
            addEdgeToG2O(mr, true, true, true);
        }

        // 6 register into map
        it->second->setPose(outPose);
        addMapFromNode(it->second, m_gridmap, getMatcher());

        // 7 reset current pose
        setParticlePoses(outPose);

        it_prev = it;
        ++it;
    }

    // 2 update keyNode pose and submap 
    updateKeyPath(); 
    reconstructKeyMap();
    rebuildLoopEdge();
    // reconstructG2O();
}

// this function must be called after updating all the pose of keynode, and node
void CGraph_SLAM::rebuildLoopEdge()
{
    multimap<int,int>::iterator it = m_key_loop.begin();
    int cnt = 0; 
    int total_cnt = m_key_loop.size(); 
    while(it != m_key_loop.end())
    {
        ROS_ERROR("graph_slam.cpp: rebuild loop edge process (%d/%d)", ++cnt, total_cnt);
        MatchingResult mr;
        CKeyNode* pknode = m_key_graph[it->first];
        CNodeScan* pnode = m_graph[it->second];
        // float score = pknode->matchNodePair(pnode, mr);
        float score = pknode->matchNodeForLoop(pnode, mr);
        if(score > m_icpReplaceThreshold)
        {
            cout<<"graph_slam.cpp: rebuild loop edge between "<<mr.id1<<" and "<<mr.id2<<endl;
            // replace this loop information 
            addEdgeToG2O(mr, true, true, true);
            // addEdgeToG2O(mr, true, false, true);
        }
        ++it;
    }
}

void CGraph_SLAM::reconstructMap()
{
    cout<<"graph_slam.cpp: start to reconstructMap()!"<<endl;
    updatePath();
    GMapping::ScanMatcherMap * new_map = new GMapping::ScanMatcherMap(m_gridmap->getCenter(),                                        m_gridmap->getWorldSizeX(), m_gridmap->getWorldSizeY(), m_gridmap->getDelta());
    getGridMap(getMatcher(), new_map);
    delete m_gridmap; 
    m_gridmap = new_map; 
    cout<<"graph_slam.cpp: finish reconstructMap()!"<<endl;
}

void CGraph_SLAM::recordCov(int id)
{
    vector<double> cov(3); 
    cov[0] = acc_cov_(0,0); 
    cov[1] = acc_cov_(1,1); 
    cov[2] = acc_cov_(2,2);
    m_cov[id] = cov;
}


bool CGraph_SLAM::exceedScore(int id, float score)
{
    if(m_score.find(id) == m_score.end())
    {
        ROS_ERROR("graph_slam.cpp: error in exceedScore no id: %d", id);
        return true;
    }
    float s = m_score[id]; 
    if(score < m_score_scale * s ) 
        return true; 
    if(s > m_minScore && score < m_minScore)
        return true;
    if(score > s)
        m_score[id] = score; 
    return false;
}

bool CGraph_SLAM::exceedCov(GMapping::OrientedPoint disp, int id)
{
    vector<double> cov = m_cov[id];
    if(fabs(disp.x) > m_cov_motion_scale * cov[0] && 
        fabs(disp.y) > m_cov_motion_scale * cov[1]) 
        {
            // ROS_WARN("graph_slam.cpp: disp.x: %f, cov[0] = %f", disp.x, cov[0]);
            return true; 
        }
    if(fabs(disp.theta) > m_cov_angle_scale)
        return true;
    return false;
}

vector<GMapping::OrientedPoint> 
CGraph_SLAM::checkPoseCov(vector<GMapping::OrientedPoint>& pose, int id)
{
    vector<GMapping::OrientedPoint> ret; 
    if(m_cov.find(id) == m_cov.end())
    {
        ROS_ERROR("graph_slam.cpp: checkPoseCov error not exist id %d", id);
        return ret;
    }
    CNodeScan* pnode = m_graph[id];
    GMapping::OrientedPoint p;
    for(int i=0; i<pose.size(); i++)
    {
        p = ominus(*(pnode->getPose()), pose[i]);
        if(!exceedCov(p, id))
            ret.push_back(pose[i]);
    }
    if(ret.size() == 0)
    {
        ROS_ERROR("graph_slam.cpp: warning, empty set of pose in checkPoseCov");
    }
    return ret;
}

bool CGraph_SLAM::addNode(CNodeScan* new_node)
{
    static ofstream outf("laser_odo.txt");
    // static int num = 0;
    if(m_graph.size() == 0)
    {
        m_pose_odo_now = m_pose_odo_pri = m_pose_last = m_pose_curr = *(new_node->getPose());
        new_node->m_id = m_graph.size();
        m_graph.insert(make_pair(new_node->m_id, new_node));
        m_loop_node.push_back(false);

     // acc_cov_ = Eigen::Matrix3d::Identity();
        acc_cov_ = Eigen::Matrix3d::Zero();

        m_cov.insert(make_pair(new_node->m_id, vector<double>(3,0.)));
        m_score[new_node->m_id] = m_minScore;

        // add reference pose into g2o
        g2o::VertexSE2 * reference_pose = new g2o::VertexSE2; 
        reference_pose->setId(0); 
        reference_pose->setEstimate(g2o::SE2());
        reference_pose->setFixed(true);
        optimizer_->addVertex(reference_pose);

        // initialize covariance 
        // memset(cur_cov_, 0, sizeof(cur_cov_));
        // cur_cov_[0] = cur_cov_[3] = cur_cov_[5] = 1;
        
   
        addMapFromNode(new_node, m_gridmap, getMatcher());
        setParticlePoses(*(new_node->getPose()));
        // outf<<++num<<" : "<<new_node->getPose()<<endl;

        // debug 
        // new_node->saveScanXY("before_map.txt", false);
        // CKeyNode* knode = new CKeyNode(new_node, m_key_graph.size()); 
        // knode->saveScanXY("after_map.txt", false);
        
        // copy for laser odo match
        if(use_laser_odo_)
        {
            // prev_x_.insert(prev_x_.begin(), new_node->m_x.begin(), new_node->m_x.end());
            // prev_y_.insert(prev_y_.begin(), new_node->m_y.begin(), new_node->m_y.end());
            prev_x_ = new_node->m_x; 
            prev_y_ = new_node->m_y;
            outf<<&m_pose_odo_now<<endl;
            // BUG detected by David Z, 2014.4.9
            // prev_r_.insert(prev_r_.begin(), new_node->m_reading.begin(), new_node->m_reading.end());
            // prev_r_ = new_node->m_reading;
        }

        return true;
    }
    new_node->m_id = m_graph.size();
    D_COUT("graph_slam.cpp: before climb-hill!");
    m_pose_last = *((m_graph[m_graph.size()-1])->getPose());
    m_pose_odo_now = *(new_node->getPose());

    if(use_laser_odo_)
    {
        double input[3] = {0}; 
        double output[3] = {0};
        // 1 caculate pose_now 
        float goodness = getICPMatcher()->ICPMatch(&prev_x_[0], &prev_y_[0], prev_x_.size(), 
            &new_node->m_x[0], &new_node->m_y[0], new_node->m_x.size(), input, output); 
        // getPLICPMatcher()->FMatch(&prev_r_[0], &(new_node->m_reading[0]), prev_r_.size(), output);
        
        // cout<<"graph_slam.cpp: prev.size() = "<<prev_r_.size()<<endl;
        // ROS_WARN("graph_slam.cpp: laser_odo: %f %f %f\n", output[0], output[1], output[2]);
        if(sqrt(SQ(output[0]) + SQ(output[1])) > 0.5 or fabs(output[2]) > 0.2)
        {
           m_pose_odo_now = m_pose_odo_pri; 
           // getICPMatcher()->setDebugWindowOn();
           //getICPMatcher()->ICPMatch(&prev_x_[0], &prev_y_[0], prev_x_.size(), 
           // &new_node->m_x[0], &new_node->m_y[0], new_node->m_x.size(), input, output); 
           // getICPMatcher()->setDebugWindowOff();
        }
        else
            m_pose_odo_now = oplus(m_pose_odo_pri, OrientedPoint(output[0], output[1], output[2]));
        outf<<&m_pose_odo_now<<endl;
    }

    for(int i=0; i<m_nposes.size(); i++)
    {
        OrientedPoint& p = m_nposes[i];
        if(i==0)
        {
            // keep the first pose == odometry 
            OrientedPoint trans = ominus(m_pose_odo_pri, m_pose_odo_now);
            p = oplus(m_nposes[i], trans);
        }else
        {
        // detected bug here David Z 3.10, mistake sequence: m_pose_odo_pri and m_pose_odo_now
            p = m_motionModel.drawFromMotion(m_nposes[i], m_pose_odo_now, m_pose_odo_pri); 
        }
    }

    /*
    // vector<OrientedPoint> nposes;
    if(!use_laser_odo_) // use odometry 
    {
        for(int i=0; i<m_nposes.size(); i++)
        {
            OrientedPoint& p = m_nposes[i];
            // detected bug here David Z 3.10, mistake sequence: m_pose_odo_pri and m_pose_odo_now
            p = m_motionModel.drawFromMotion(m_nposes[i], m_pose_odo_now, m_pose_odo_pri); 
        }
    }else  // use laser scan as odometry
    {
        double input[3] = {0}; 
        double output[3] = {0};
        // 1 caculate pose_now 
        float goodness = getICPMatcher()->ICPMatch(&prev_x_[0], &prev_y_[0], prev_x_.size(), 
            &new_node->m_x[0], &new_node->m_y[0], new_node->m_x.size(), input, output); 
        // getPLICPMatcher()->FMatch(&prev_r_[0], &(new_node->m_reading[0]), prev_r_.size(), output);
        
        // cout<<"graph_slam.cpp: prev.size() = "<<prev_r_.size()<<endl;
        if(fabs(output[0]) >= 0.5 or fabs(output[1]) >= 0.5)
        {
            m_pose_odo_now = m_pose_odo_pri; 
            goodness = 0;
        }else
            m_pose_odo_now = oplus(m_pose_odo_pri, OrientedPoint(output[0], output[1], output[2]));
        

        // 2 apply cov from icp to simulate motionModel 
        double p[3]; 
        double cov[6];
        getICPMatcher()->getResult(p, cov);
        
        GMapping::MotionModel laser_motion = m_motionModel; 
        float scale = 10; 
        laser_motion.srr= (cov[5]) + m_motionModel.srr * (1-goodness) * scale; // srr;
        laser_motion.srt= ((cov[2] + cov[4])/2.) + m_motionModel.srt * (1-goodness) * scale; // srt;
        laser_motion.str= laser_motion.srt ; // str;
        laser_motion.stt= ((cov[0] + cov[3])/2.) + m_motionModel.stt * (1-goodness) * scale; // stt;
        // cout<<"graph_slam.cpp: icp goodness: "<<goodness<<endl;
        // cout<<"graph_slam.cpp: motion cov x: "<<cov[0]<<" y: "<<cov[3]<<" theta: "<<cov[5]<<endl;
        cout<<"graph_slam.cpp: ICP result: "<<output[0]<<" "<<output[1]<<" "<<R2D(output[2])<<endl;
        ROS_ERROR("goodness : %f, cov x: %f, cov y: %f, cov theta: %f", goodness, cov[0], cov[3], cov[5]);
        
        for(int i=0; i<m_nposes.size(); i++)
        {
            OrientedPoint& p = m_nposes[i];
            p = laser_motion.drawFromMotion(m_nposes[i], m_pose_odo_now, m_pose_odo_pri); 
        }
    }*/
    
    // copy for laser odo match
    if(use_laser_odo_)
    {
        // prev_x_.insert(prev_x_.begin(), new_node->m_x.begin(), new_node->m_x.end());
        // prev_y_.insert(prev_y_.begin(), new_node->m_y.begin(), new_node->m_y.end());
        // prev_r_.insert(prev_r_.begin(), new_node->m_reading.begin(), new_node->m_reading.end());
        prev_x_ = new_node->m_x; 
        prev_y_ = new_node->m_y; 
        // prev_r_ = new_node->m_reading;
    }

    // accumulate the robot translation and rotation
    OrientedPoint move = m_pose_odo_now - m_pose_odo_pri;
    move.theta=atan2(sin(move.theta), cos(move.theta));
    linearUpdate_ += sqrt(move*move);
    angularUpdate_ += fabs(move.theta);
    m_pose_odo_pri = m_pose_odo_now;

    float score;
    if(linearUpdate_ > m_linearUpdate || \
        angularUpdate_ > m_angularUpdate)
    {
        // 1 climb-hill to predict a possible pose  
        OrientedPoint iniPose = MRMC_ClimbHill(new_node, score);
        if(score < 120) //m_minScore)
        {
            iniPose = m_nposes[0];
            // TODO: set it as odo pose
        }
        // OrientedPoint iniPose = *(new_node->getPose()); // without MRMC

        // 2 use icp to obtain covariance 
        MatchingResult mr;
        ICP_PairWise(new_node, iniPose, m_graph.size()-1, m_pose_curr, mr, score);
        
        // m_pose_curr = ICP_PairWise(new_node, iniPose);
        // m_pose_curr = iniPose;

        // 3 add into g2o 
        addEdgeToG2O(mr, true, true);

        // 4 insert into graph 
        new_node->setPose(m_pose_curr);
        m_graph[new_node->m_id] = new_node;
        m_loop_node.push_back(false); // add this node
        recordCov(new_node->m_id);
        m_score[new_node->m_id] = score;

        // 5 detect loop        
        /*
        vector<int> potential_nodes = getPotentialLoopNodes(new_node);
        bool foundLoop = false;
        for(int i=0; i<potential_nodes.size(); i++)
        {
            MatchingResult mr; 
            OrientedPoint loopPose;
            if(ICP_PairWise(new_node, m_pose_curr, potential_nodes[i], loopPose, mr, true))
            {
                cout<<"graph_slam.cpp: add loop edge between "<<mr.id1<<" and "<<mr.id2<<endl;
                addEdgeToG2O(mr, true, false);
                foundLoop = true; 
            }
        }
        
        if(foundLoop)
        {
            optimizeGraph(1);
            calMarginalCov(new_node->m_id); // update current motion cov using marginal cov
            g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(new_node->m_id));
            double t[3]; 
            pVertex->getEstimateData(t); 
            GMapping::OrientedPoint tmp(t[0], t[1], t[2]);
            new_node->setPose(tmp);
            // new_node->setPose(GMapping::OrientedPoint(t[0], t[1], t[2]));
        }*/
        
        bool foundLoop = false; 
        vector<int> potential_nodes = getPotentialLoopKeyNodes(new_node);
        float best_score = -1.;
        int best_knode_id = -1;
        int best_node_id = -1;
        for(int i=0; i<potential_nodes.size(); i++)
        {
            MatchingResult mr;
            CKeyNode* pknode = m_key_graph[potential_nodes[i]];
            float score; 
            if(1 || use_laser_odo_)
            {
                score = pknode->matchNodeForLoop(new_node, mr);
            }else
            {
                score = pknode->matchNodePair(new_node, mr);
            }
            ROS_ERROR("graph_slam.cpp: detected keynode %d, with node %d, score: %f", pknode->k_id_, new_node->m_id, score);

            if(score > m_icpReplaceThreshold && isLoopInCov(mr))
            {
                cout<<"graph_slam.cpp: add loop edge between "<<mr.id1<<" and "<<mr.id2<<endl;
                addEdgeToG2O(mr, true, true);
                // addEdgeToG2O(mr, true, false);
                foundLoop = true; 
                m_key_loop.insert(make_pair(pknode->k_id_, new_node->m_id)); // used for rebuild loop edge
                m_loop_node[new_node->m_id] = true; // whether this node matched for loop
                if(score > best_score)
                {
                    best_score = score ;
                    best_knode_id = potential_nodes[i];
                    best_node_id = mr.id1;
                }
            }
        }
        if(foundLoop)
        {
            // cout<<"graph_slam.cpp: try to switch key node for loop!"<<endl;
            ROS_ERROR("graph_slam.cpp: try to switch key node for loop!");
            // switch current keynode and optimization

            // 1 register current node
            // registerKeyNode(current_key_node_);    

            // add last node into current node
            // MatchingResult mr;  
            // if(current_key_node_->matchNodePair(new_node, mr) > 0)
            {
               // addEdgeToG2O(mr, true, false);
            }
       
            g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(new_node->m_id));
            double t[3]; 
            pVertex->getEstimateData(t); 
            GMapping::OrientedPoint tmp(t[0], t[1], t[2]);
            GMapping::OrientedPoint dis_p = ominus(*(new_node->getPose()), tmp); 
            ROS_ERROR("graph_slam.cpp: before optimization, curr_pose change:"); 
            cout<<&dis_p<<endl;

            // 2 optimization 
            saveG2O("before_rebuild.g2o");
            optimizeGraph();
            // reconstructMap();  // this will be deleted!

            if(sqrt(SQ(dis_p.x) + SQ(dis_p.y)) > pose_dis_thresh_) // this should be parameterized
            // if(fabs(dis_p.x) > 0.5 || fabs(dis_p.y) > 0.5)
            {
                // saveG2O("before_rebuild.g2o");
                // int_info("graph_slam.cpp: big_pose, start to reconstruct map!");
                // rebuildMapAndG2O();
                if(use_iterative_g2o)
                {
                    iterativeOptGraph();
                }
                reconstructMap(); 
                updateKeyPath();
                reconstructKeyMap();

                // ROS_ERROR("graph_slam.cpp look at me, for reconstructMap with large dis");
            }  
            calMarginalCov(new_node->m_id);
            balanceCov(best_node_id);
            recordCov(new_node->m_id);

            // 3 switch to new key node
            current_key_node_ = m_key_graph[best_knode_id];
            current_key_node_->setActive();

            // 4 update current pose
            /*g2o::VertexSE2**/
            pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(new_node->m_id));
            pVertex->getEstimateData(t); 
            tmp = GMapping::OrientedPoint(t[0], t[1], t[2]);
            new_node->setPose(tmp);
            m_pose_curr = tmp;

            // 5 reduce this node into keynode
            current_key_node_->addNode(new_node);

        }
        else
        {
            // try keynode 
            if(current_key_node_ == 0) // first key_node
            {
                cout<<"graph_slam.cpp: create first keynode!"<<endl;
                current_key_node_ = new CKeyNode(new_node, m_key_graph.size());
                m_key_graph.insert(make_pair(current_key_node_->k_id_, current_key_node_));
            }
            else if(sqrDisPose(new_node->getPose(), current_key_node_->getPose()) > sqr_dis_thresh_) // start a new key node
            {
                cout<<"graph_slam.cpp: try to switch keyNode for distance!"<<endl;
                cout<<"last_key_node: "<<current_key_node_->getPose()<<endl; 
                cout<<"curr_node: "<<new_node->getPose()<<endl;
                // switch to a new keyNode 

                // 1 register cur key node
                // registerKeyNode(current_key_node_); 

                current_key_node_ = new CKeyNode(new_node, m_key_graph.size());
                m_key_graph.insert(make_pair(current_key_node_->k_id_, current_key_node_));
                cout<<"graph_slam.cpp: key_graph size: "<<m_key_graph.size()<<endl;
                // int_info("graph_slam.cpp: after a new KeyNode, watch memory!");
            }else
            {
                // this node belongs to current key_node
                current_key_node_->addNode(new_node);
            }
        }
        
        // cout<<"graph_slam.cpp: try to register scan into map"<<endl;

        // 6 register scan into map
        addMapFromNode(new_node, m_gridmap, getMatcher());
        setParticlePoses(m_pose_curr);

        // outf<<"output : "<<&m_pose_curr<<endl;

        linearUpdate_ = 0;
        angularUpdate_ = 0;
    }else
    {
        return false;
    }
    return true;
}

void CGraph_SLAM::registerKeyNode(CKeyNode* pknode, bool reset)
{
    CNodeScan* pnode;
    float score; 
    int cnt = 0;
    // i==0 is keynode itself
    // pknode->saveScanXY("n1_scan.txt", false);
    // pknode->saveScanXY();

    for(int i=1; i<pknode->nodes_.size(); i++)
    {
        if(pknode->b_register_[i]) continue; 
        MatchingResult mr; 
        pnode = pknode->nodes_[i];
        // stringstream ss;
        // ss<<"n"<<pnode->m_id<<"_scan.txt";
        // pnode->saveScanXY(ss.str().c_str(), false);
        // pnode->saveScanXY();
        score = pknode->matchNodePair(pnode, mr); 
        if(score > m_icpReplaceThreshold)
        {
            addEdgeToG2O(mr, true, true, reset);// false); 
            cnt++;
        }
        pknode->b_register_[i] = true;
        // cout<<"graph_slam.cpp: match node: "<<pnode->m_id<<" with key_node: "<<pknode->k_id_<<" score: "<<score<<endl;
    }
    cout<<"graph_slam.cpp: registerKeyNode has "<<cnt<<" edges total : "<<pknode->nodes_.size()<<endl;
}

double CGraph_SLAM::sqrDisPose(GMapping::OrientedPoint* p1, GMapping::OrientedPoint* p2)
{
    return SQ((p1->x)- (p2->x)) + SQ((p1->y) - (p2->y)); 
}

void CGraph_SLAM::reconstructG2O()
{
    key_iterator kit = kbegin();
    while(kit != kend())
    {
        kit->second->resetRegister();
        registerKeyNode(kit->second, true);
        ++kit;
    }
}

double CGraph_SLAM::iterativeOptGraph(int iter)
{
    static bool once = true; 
    if(current_key_node_ != 0 && once)
    {
        // registerKeyNode(current_key_node_);
        once = false;
    }
    double last_chi2 = optimizeGraph(iter); 
    double curr_chi2 ;
    // optimizer_->save("before_rebuild.g2o");
    int N = 3; 
    ros::NodeHandle nh("~");
    if(!nh.getParam("iter_op_num", N))
        N = 5;
    int i =0;
    if(iter > 0) N = iter;
    while(i++ < N )
    {
        // reconstruct Map and g2o 
        // reconstructMap(); // update path and reconstruct global map 
        rebuildMapAndG2O();
        // reconstructKeyMap();
        // reconstructG2O(); // reconstruct G2O edges, now just for key node
        curr_chi2 = optimizeGraph(iter);
        ROS_ERROR("graph_slam.cpp: last chi2: %f, curr chi2: %f", last_chi2, curr_chi2);
        if(fabs( last_chi2 - curr_chi2) < 1e-2 /*|| curr_chi2 > last_chi2*/)
        {
            ROS_ERROR("graph_slam.cpp: break iterative_optimization, last_chi2: %f, curr_chi2: %f", last_chi2, curr_chi2);
            break;
        }
        /* if(curr_chi2 >= last_chi2 || ( last_chi2 - curr_chi2) < 1e-5)
        {
            ROS_ERROR("graph_slam.cpp: break iterative_optimization, last_chi2: %f, curr_chi2: %f", last_chi2, curr_chi2);
            break;
        }*/
        last_chi2 = curr_chi2;
    }
    // after iteratively rebuild and optimization, we have to make consistence
    reconstructMap(); 
    updateKeyPath();
    reconstructKeyMap();

    // optimizer_->save("after_build.g2o");
    cout<<"graph_slam.cpp: finish iterativeOptGraph()!"<<endl;
    return curr_chi2;
}

double CGraph_SLAM::optimizeGraph(int iter)
{
    // This should be parameterized
    static int n_iter=10;
    if(iter<=0) iter = n_iter;
    optimizer_->initializeOptimization();
    static double error_step_threshold = 0.00001;
    double last_err, cur_err;
    last_err = -1;
    for(int i=0; i<iter; i++)
    {
        optimizer_->optimize(1);
        optimizer_->computeActiveErrors();
        cur_err = optimizer_->activeChi2();
        if(last_err >= 0)
        {
            if(fabs(last_err - cur_err)<error_step_threshold)
                break;
        }    
        last_err = cur_err;
    }  
    return cur_err;
}

void CGraph_SLAM::balanceCov(int loop_id)
{
    map<int, vector<double> >::iterator it_cov = m_cov.find(loop_id);
    if(it_cov == m_cov.end())
    {
        ROS_ERROR("graph_slam.cpp: failed to find loop_id: %d", loop_id);
        return ;
    }
    ros::NodeHandle nh("~"); 
    double cov_fuse_alpha; 
    if(!nh.getParam("cov_fuse_alpha", cov_fuse_alpha))
        cov_fuse_alpha = 0.5; 
    acc_cov_(0,0) = cov_fuse_alpha * acc_cov_(0,0) + (1-cov_fuse_alpha) * it_cov->second[0]; 
    acc_cov_(1,1) = cov_fuse_alpha * acc_cov_(1,1) + (1-cov_fuse_alpha) * it_cov->second[1]; 
    acc_cov_(2,2) = cov_fuse_alpha * acc_cov_(2,2) + (1-cov_fuse_alpha) * it_cov->second[2]; 
    ROS_WARN("graph_slam.cpp: after balance, marginal_cov: %f %f", acc_cov_(0,0),acc_cov_(1,1));
}

void CGraph_SLAM::calMarginalCov(int id) 
{
    g2o::OptimizableGraph::Vertex * v = optimizer_->vertex(id);
    g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv; 
    /*if(v->hessianIndex() < 0)
      {
      cout<<"graph_slam.cpp: v->hessianIndex = "<<v->hessianIndex()<<endl;
      return ;
      }
      std::vector<std::pair<int, int> > index; 
      index.push_back(std::pair<int, int>(v->hessianIndex(), v->hessianIndex()));
      optimizer_->computeMarginals(spinv, index);*/
    if(!optimizer_->computeMarginals(spinv, v)) 
    {   
        cout<<"graph_slam.cpp: this operation is not supported by this solver!"<<endl;
        return ;
    }else
    {
         // cout<<"covariance\n"<<spinv<<endl;
         // cout<<spinv.block(0,0)<<endl;
         ROS_WARN("graph_slam.cpp: before loop optimization, acc_cov: %f %f", acc_cov_(0,0), acc_cov_(1,1));
         for(size_t i=0; i<spinv.blockCols().size(); i++)
         {
             for(g2o::SparseBlockMatrix<Eigen::MatrixXd>::IntBlockMap::iterator it = spinv.blockCols()[i].begin(); it!= spinv.blockCols()[i].end(); ++it)
             {
                 g2o::SparseBlockMatrix<Eigen::MatrixXd>::SparseMatrixBlock* m = it->second; 
                 // cout<<"CACACA: "<<it->first<<" "<<i<<endl;
                 Eigen::Matrix3d cov_Marginal; 
                 // cout<<"cacaca\n "<<*m<<endl;
                 for(int i=0 ;i<m->rows(); i++)
                 {
                     for(int j=0; j<m->cols(); j++) 
                         cov_Marginal(i,j) = (*m)(i,j);
                 }
                 acc_cov_ = cov_Marginal.inverse(); 
                 // acc_cov_ = cov_Marginal; 
                 break; // actually it only has been executed once
             }
         }
    }
    ROS_WARN("graph_slam.cpp: after loop optimization, marginal_cov: %f %f", acc_cov_(0,0),acc_cov_(1,1));
}

bool CGraph_SLAM::addEdgeToG2O(MatchingResult& mr, bool large_edge, bool set_estimate, bool replace)
{
    g2o::VertexSE2 * v1 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(mr.id1));
    g2o::VertexSE2 * v2 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(mr.id2));

    // assert the transformation is large enough to avoid too many vertices on the same spot
    if(!v1 || !v2){
        if(!large_edge){
            return false;
        }
    }
    if(!v1 && !v2){
        std::cout<<"both the nodes are not in the graph!"<<std::endl;
        return false;
    }
    else if(v1 && !v2){
        v2 = new g2o::VertexSE2;
        assert(v2);
        v2->setId(mr.id2);
        g2o::SE2 t = v1->estimate()*mr.m_edge.measurement();
        v2->setEstimate(t);
        optimizer_->addVertex(v2);
    }else if(v2 && !v1){
        v1 = new g2o::VertexSE2;
        assert(v1);
        v1->setId(mr.id1);
        // g2o::SE2 t = v2->estimate()*mr.m_edge.inverseMeasurement();
        g2o::SE2 t = v2->estimate()*mr.m_edge.measurement().inverse();
        v1->setEstimate(t);
        optimizer_->addVertex(v1);
    }else 
    {
        // v1 && v2 exist
        if(set_estimate)
        {
            g2o::SE2 t = v1->estimate()*mr.m_edge.measurement();
            v2->setEstimate(t);
        }
        if(replace)
        {
            bool found_edge = false;
            // find this edge
            for(g2o::HyperGraph::EdgeSet::const_iterator it = v2->edges().begin(); it!=v2->edges().end(); ++it)
            {
                if((*it)->vertices()[0]->id() == v1->id())
                {
                    g2o::EdgeSE2* get_edge = dynamic_cast<g2o::EdgeSE2*>(*it);
                    get_edge->setMeasurement(mr.m_edge.measurement());
                    get_edge->setInformation(mr.m_edge.information());
                    found_edge = true;
                    break;
                }
            }
            if(found_edge)
            {
                return true;
            }else
            {
                ROS_ERROR("graph_slam.cpp: failed to find edge between %d and %d!", v1->id(), v2->id());
            }
        }
    }
    g2o::EdgeSE2 * g2o_edge = new g2o::EdgeSE2;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    g2o_edge->setMeasurement(mr.m_edge.measurement());
    g2o_edge->setInformation(mr.m_edge.information());
    // g2o_edge->setInverseMeasurement(mr.m_edge.inverseMeasurement());
    optimizer_->addEdge(g2o_edge);
    return true;
}

void CGraph_SLAM::getKeyNodeGridMap(GMapping::ScanMatcher& matcher, GMapping::ScanMatcherMap* gridmap)
{
    if(current_key_node_ == 0)
    {
        addMapFromNode(begin()->second, gridmap, matcher);
    }else
    {
        for(int i=0; i<current_key_node_->nodes_.size(); i++)
        {
            addMapFromNode(current_key_node_->nodes_[i], gridmap, matcher); 
        }
    }
    return ;
}


void CGraph_SLAM::getGridMap(GMapping::ScanMatcher& matcher, GMapping::ScanMatcherMap* gridmap)
{
    iterator it = begin();
    while(it != end())
    {
        addMapFromNode(it->second, gridmap, matcher);
        ++it;
    }
}

void CGraph_SLAM::setSensorMap(GMapping::SensorMap& smap)
{
    SensorMap::const_iterator laser_it=smap.find(std::string("FLASER"));
    if (laser_it==smap.end())
    {
        cerr << "Attempting to load the new carmen log format" << endl;
        laser_it=smap.find(std::string("ROBOTLASER1"));
        assert(laser_it!=smap.end());
    }     
    const RangeSensor* rangeSensor=dynamic_cast<const RangeSensor*>((laser_it->second));
    assert(rangeSensor && rangeSensor->beams().size());

    unsigned int m_beams=static_cast<unsigned int>(rangeSensor->beams().size());
    double* angles=new double[rangeSensor->beams().size()];
    for (unsigned int i=0; i<m_beams; i++)
    {
        angles[i]=rangeSensor->beams()[i].pose.theta;
    }     
    getMatcher().setLaserParameters(m_beams, angles, rangeSensor->getPose());
    delete [] angles;
}

void CGraph_SLAM::init(double xmin, double ymin, double xmax, double ymax, double delta,
    OrientedPoint initialPose)
{
    // ScanMatcher lmap(Point(xmin+xmax, ymin+ymax)*.5, xmax - xmin, ymax - ymin, delta);
    // *m_gridmap = lmap;
    m_gridmap = new ScanMatcherMap(Point(xmin+xmax, ymin+ymax)*.5, xmax - xmin, ymax - ymin, delta);
}

void CGraph_SLAM::setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma, double likelihoodGain, unsigned int likelihoodSkip)
{
    getMatcher().setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
}

void CGraph_SLAM::setGenerateMap(bool b_gen_Map)
{
    getMatcher().setgenerateMap(b_gen_Map);
}

void CGraph_SLAM::setMotionModelParameters(double srr, double srt, double str, double stt)
{
    m_motionModel.srr=srr;
    m_motionModel.srt=srt;
    m_motionModel.str=str;
    m_motionModel.stt=stt;
}

void CGraph_SLAM::writePath(const char* fname)
{
    ros::NodeHandle private_nh_("~");
    string save_path;
    if(!private_nh_.getParam("save_path", save_path))
        save_path = "";
    string save_file = save_path + "/" + string(fname);
    // cout<<"graph_slam.cpp: fname: "<<fname<<" save_path: "<<save_path<<endl;
    // cout<<"graph_slam.cpp: save_file: "<<save_file<<endl;
    ofstream ouf(save_file.c_str());
    if(!ouf.is_open())
    {
        cout<<"graph_slam.cpp: failed to open file: "<<fname<<endl;
        return ;
    }
    map<int, CNodeScan*>::iterator it = m_graph.begin();
    while( it!= m_graph.end())
    {
        ouf<<it->second->getPose()<<endl;
        ++it;
    }
}

void testOptimizer(const char* fname)
{
    CGraph_SLAM* pg = new CGraph_SLAM(); 
    pg->optimizer_->load(fname); 
    pg->optimizeGraph(); 
    pg->optimizer_->save("test.g2o");
    cout<<"graph_slam.cpp: testOptimizer finished, save into test.g2o"<<endl;
}

