#include "graph_slam.h"
#include "debug.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "ZHIcp_Warpper.h"
#include "point.h"

#include "g2o/math_groups/se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/hyper_dijkstra.h" 
#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "MatchingResult.h"

using namespace GMapping;

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1,-1> > SlamBlockSolver; 
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver; 
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver; 

typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*> VertexIDMap;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

CGraph_SLAM::CGraph_SLAM(double minScore, int nParticles): 
m_nMRMC(nParticles),
m_minScore(minScore),
m_gridmap(0), 
m_icp(new CICPWarpper),
optimizer_(0)
{
    assert(m_nMRMC > 0);
    m_nposes.resize(m_nMRMC);
    m_icp->initICP();
#ifndef ONLINE
    optimizer_ = new g2o::SparseOptimizer();
    SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
    linearSolver->setBlockOrdering(false); 
    SlamBlockSolver * solver = new SlamBlockSolver(optimizer_, linearSolver); 
    optimizer_->setSolver(solver);

#else
    optimizer_ = new g2o::SparseOptimizerIncremental();
    optimizer_->initSolver(3.);
#endif
    optimizer_->setVerbose(true);
}

CGraph_SLAM::~CGraph_SLAM()
{
    if(m_gridmap) delete m_gridmap;
    if(m_icp) delete m_icp;
    if(optimizer_) delete optimizer_;
}

void CGraph_SLAM::addMapFromNode(CNodeScan* new_node, ScanMatcherMap* gridmap, ScanMatcher& matcher)
{
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(*gridmap, *(new_node->getPose()), new_node->getPlainReading());
    // matcher.invalidateActiveArea();
    matcher.registerScan(*gridmap,  *(new_node->getPose()), new_node->getPlainReading());
}

void CGraph_SLAM::setParticlePoses(GMapping::OrientedPoint& p)
{
    for(int i=0; i<m_nposes.size(); i++)
    {
        m_nposes[i] = p; 
    }
}

GMapping::OrientedPoint CGraph_SLAM::MRMC_ClimbHill(CNodeScan* new_node)
{
    if(m_graph.size()<=0) 
    {
        cout<<"graph_slam.cpp: map not be intialized!"<<endl;
        return GMapping::OrientedPoint();
    }
    double tx, ty, ttheta, ts;
    tx = ty = ttheta = ts = 0;
    double maxScore = m_minScore;
    for(int i=0; i<m_nposes.size(); i++)
    {
        OrientedPoint corrected; 

        D_COUT("graph_slam.cpp: before optimize!");
        D_COUT("graph_slam.cpp: input pose: ");

        // outf<<"input "<< i<<" : "<<&m_nposes[i]<<endl;
        /*outf<<++num<<" input: "<<&m_pose_odo_now<<endl;
          for(int j=0; j<new_node->m_reading.size(); j++)
          {
          outf<<new_node->m_reading[j]<<" ";
          if(j!=0 && j%15==0) outf<<endl;
          }
          outf<<endl;*/
        double score = m_matcher.optimize(corrected, *m_gridmap, m_nposes[i], &(new_node->m_reading[0]));
        // outf<<"ouput: "<<i<<" : "<<&corrected<<endl;
        D_COUT2("graph_slam.cpp: after optimize! score = ", score)
            // cout<<"graph_slam.cpp: after optimize! score = "<<score<<endl;
            if(score < m_minScore)
            {
                score = m_minScore; 
                corrected = *(new_node->getPose());
            }else
            {
                tx += corrected.x * score; 
                ty += corrected.y * score; 
                ttheta += corrected.theta * score;  
            }

        /*
           if(score > maxScore)
           {
           tx = corrected.x;
           ty = corrected.y;
           ttheta = corrected.theta;
           maxScore = score;
           }*/
        // tx += corrected.x * score; 
        // ty += corrected.y * score; 
        // ttheta += corrected.theta * score;  
        ts += score; 
    }
    OrientedPoint iniPose(tx/ts, ty/ts, ttheta/ts);
    return iniPose;
}

GMapping::OrientedPoint CGraph_SLAM::ominus(GMapping::OrientedPoint& p1, GMapping::OrientedPoint& p2)
{
    OrientedPoint2D pd1(p1.x, p1.y, p1.theta); 
    OrientedPoint2D pd2(p2.x, p2.y, p2.theta); 
    OrientedPoint2D rel = pd1.ominus(pd2);
    return GMapping::OrientedPoint(rel.x, rel.y, rel.theta);
}

GMapping::OrientedPoint CGraph_SLAM::ICP_PairWise(CNodeScan* new_node, GMapping::OrientedPoint& iniPose)
{
    CNodeScan* prev_node = m_graph[m_graph.size()-1]; 
    GMapping::OrientedPoint last_pose = *(prev_node->getPose());
    OrientedPoint2D last_pose_2d( last_pose.x, last_pose.y, last_pose.theta); 
    OrientedPoint2D curr_pose_2d( iniPose.x, iniPose.y , iniPose.theta);
    OrientedPoint2D rel_ini_2d = last_pose_2d.ominus(curr_pose_2d);

    double input_p[3] = {rel_ini_2d.x, rel_ini_2d.y, rel_ini_2d.theta}; 
    double output_p[3] = {0}; 
    int num =  CNodeScan::N_BEAMS;
    float goodness = m_icp->ICPMatch(&prev_node->m_x[0], &prev_node->m_y[0], 
                                     &new_node->m_x[0], &new_node->m_y[0], num ,
                                     input_p, output_p); 
    if(goodness > m_icpGoodness)
    {
        cout<<"graph_slam.cpp: replace with ICP goodness: "<<goodness<<endl; 
        OrientedPoint2D rel_icp(output_p[0], output_p[1], output_p[2]); 
        curr_pose_2d = last_pose_2d.oplus(rel_icp);
        return GMapping::OrientedPoint(curr_pose_2d.x, curr_pose_2d.y, curr_pose_2d.theta); 
    }else
    {
        cout<<"graph_slam.cpp: poor icp goodness: "<<goodness<<endl;
    }
    return iniPose;
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
    ret.setInverseMeasurement(tmpSE.inverse());
    return ;
}

bool CGraph_SLAM::addNode(CNodeScan* new_node)
{
    // static ofstream outf("graph_slam.txt");
    // static int num = 0;
    if(m_graph.size() == 0)
    {
        m_pose_odo_now = m_pose_odo_pri = m_pose_last = m_pose_curr = *(new_node->getPose());
        new_node->m_id = m_graph.size();
        m_graph.insert(make_pair(new_node->m_id, new_node));
        
        // add reference pose into g2o
        g2o::VertexSE2 * reference_pose = new g2o::VertexSE2; 
        reference_pose->setId(0); 
        reference_pose->setEstimate(g2o::SE2());
        reference_pose->setFixed(true);
        optimizer_->addVertex(reference_pose);

        addMapFromNode(new_node, m_gridmap, m_matcher);
        setParticlePoses(*(new_node->getPose()));
        // outf<<++num<<" : "<<new_node->getPose()<<endl;
        return true;
    }
    new_node->m_id = m_graph.size();
    // cout<<"graph_slam.cpp: before climb-hill!"<<endl;
    D_COUT("graph_slam.cpp: before climb-hill!");
    // m_pose_last = *((m_graph[m_graph.size()-1])->getPose());
    m_pose_odo_now = *(new_node->getPose());
    // vector<OrientedPoint> nposes;
    for(int i=0; i<m_nposes.size(); i++)
    {
        OrientedPoint& p = m_nposes[i];
        // detected bug here David Z 3.10, mistake sequence: m_pose_odo_pri and m_pose_odo_now
        p = m_motionModel.drawFromMotion(m_nposes[i], m_pose_odo_now, m_pose_odo_pri); 
    }
    // accumulate the robot translation and rotation
    OrientedPoint move = m_pose_odo_now - m_pose_odo_pri;
    move.theta=atan2(sin(move.theta), cos(move.theta));
    linearUpdate_ += sqrt(move*move);
    angularUpdate_ += fabs(move.theta);
    m_pose_odo_pri = m_pose_odo_now;
    
    if(linearUpdate_ > m_linearUpdate || \
        angularUpdate_ > m_angularUpdate)
    {
        // 1 climb-hill to predict a possible pose  
        OrientedPoint iniPose = MRMC_ClimbHill(new_node);

        // 2 use icp to precisely align 
        m_pose_curr = ICP_PairWise(new_node, iniPose);
        m_pose_curr = iniPose;

        // 3 add into g2o 
        CNodeScan* pre_node = m_graph[m_graph.size()-1]; 

        MatchingResult mr ;
        mr.id1 = pre_node->m_id; 
        mr.id2 = new_node->m_id; 
        OrientedPoint transPose = ominus(*(pre_node->getPose()), m_pose_curr);
        fromOrientedPoint2SE2(transPose, mr.m_edge);
        mr.m_edge.setInformation(Eigen::Matrix3d::Identity());
        addEdgeToG2O(mr, true, true);

        // 4 insert into graph 
        new_node->m_id = m_graph.size();
        new_node->setPose(m_pose_curr);
        m_graph[new_node->m_id] = new_node;
        
        // 5 register scan into map
        addMapFromNode(new_node, m_gridmap, m_matcher);
        setParticlePoses(m_pose_curr);
        
        // outf<<"output : "<<&m_pose_curr<<endl;

        linearUpdate_ = 0;
        angularUpdate_ = 0;
    }else{
        return false;
    }
    return true;
}

bool CGraph_SLAM::addEdgeToG2O(MatchingResult& mr, bool large_edge, bool set_estimate)
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
        g2o::SE2 t = v2->estimate()*mr.m_edge.inverseMeasurement();
        v1->setEstimate(t);
        optimizer_->addVertex(v1);
    }else {
        if(set_estimate){
            g2o::SE2 t = v1->estimate()*mr.m_edge.measurement();
            v2->setEstimate(t);
        }
    }
    g2o::EdgeSE2 * g2o_edge = new g2o::EdgeSE2;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    g2o_edge->setMeasurement(mr.m_edge.measurement());
    g2o_edge->setInformation(mr.m_edge.information());
    g2o_edge->setInverseMeasurement(mr.m_edge.inverseMeasurement());
    optimizer_->addEdge(g2o_edge);
    return true;
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
    if (laser_it==smap.end()){
        cerr << "Attempting to load the new carmen log format" << endl;
        laser_it=smap.find(std::string("ROBOTLASER1"));
        assert(laser_it!=smap.end());
    }     
    const RangeSensor* rangeSensor=dynamic_cast<const RangeSensor*>((laser_it->second));
    assert(rangeSensor && rangeSensor->beams().size());

    unsigned int m_beams=static_cast<unsigned int>(rangeSensor->beams().size());
    double* angles=new double[rangeSensor->beams().size()];
    for (unsigned int i=0; i<m_beams; i++){
        angles[i]=rangeSensor->beams()[i].pose.theta;
    }     
    m_matcher.setLaserParameters(m_beams, angles, rangeSensor->getPose());
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
    m_matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
}

void CGraph_SLAM::setGenerateMap(bool b_gen_Map)
{
    m_matcher.setgenerateMap(b_gen_Map);
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

