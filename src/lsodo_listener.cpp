#include "lsodo_listener.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"
#include "graph_slam.h"
#include "debug.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

using namespace std;

LSODO_Listener::LSODO_Listener() : 
m_map_to_odom(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  m_laser_count(0), m_transform_thread(NULL), m_got_map(false), 
  m_pGraphSlam(0), 
  m_gotFirstScan(false)
{
    m_tfB = new tf::TransformBroadcaster(); 
    ROS_ASSERT(m_tfB);
    // tf frames 
    ros::NodeHandle private_nh_("~"); 
    
    // graph parameters
    double minScore ;
    int nMCMC; 
    private_nh_.param("min_score", minScore, 295.); 
    private_nh_.param("nMCMC", nMCMC, 30);
    m_pGraphSlam = new CGraph_SLAM(minScore, nMCMC);

    // mapUpdate interval
    double tmp;
    if(!private_nh_.getParam("map_update_interval", tmp))
        tmp = 5.0;
    m_update_interval.fromSec(tmp);

    // tf broadcast 
    double transform_publish_period; 
    private_nh_.param("transform_publish_period", transform_publish_period, 0.05); 
    if(!private_nh_.getParam("tf_delay", m_tf_delay))
        m_tf_delay = transform_publish_period;

    if(!private_nh_.getParam("base_frame", m_base_frame))
        m_base_frame = "base_link"; 
    if(!private_nh_.getParam("map_frame", m_map_frame))
        m_map_frame = "map"; 
    if(!private_nh_.getParam("odom_frame", m_odom_frame))
        m_odom_frame = "odom";
    if(!private_nh_.getParam("throttle_scans", m_throttle_scans))
        m_throttle_scans = 1;

    // linear update 
    if(!private_nh_.getParam("linearUpdate", linearUpdate_))
        linearUpdate_ = 0.5 ;
    // angular update 
    if(!private_nh_.getParam("angularUpdate", angularUpdate_))
        angularUpdate_ = 0.2;

    // map parameters
    if(!private_nh_.getParam("delta", m_delta_))
        m_delta_ = 0.05;
    if(!private_nh_.getParam("xmin", xmin_))
        xmin_ = -100.0;
    if(!private_nh_.getParam("ymin", ymin_))
        ymin_ = -100.0;
    if(!private_nh_.getParam("xmax", xmax_))
        xmax_ = 100.0;
    if(!private_nh_.getParam("ymax", ymax_))
        ymax_ = 100.0;
    if(!private_nh_.getParam("delta", m_delta_))
        m_delta_ = 0.05;
    if(!private_nh_.getParam("occ_thresh", occ_thresh_))
        occ_thresh_ = 0.25;
    if(!private_nh_.getParam("sigma", sigma_))
        sigma_ = 0.05;
    if(!private_nh_.getParam("kernelSize", kernelSize_))
        kernelSize_ = 1;
    if(!private_nh_.getParam("lstep", lstep_))
        lstep_ = 0.05;
    if(!private_nh_.getParam("astep", astep_))
        astep_ = 0.05;
    if(!private_nh_.getParam("iterations", iterations_))
        iterations_ = 5;
    if(!private_nh_.getParam("lsigma", lsigma_))
        lsigma_ = 0.075;
    if(!private_nh_.getParam("ogain", ogain_))
        ogain_ = 3.0;
    if(!private_nh_.getParam("lskip", lskip_))
        lskip_ = 0;
    if(!private_nh_.getParam("srr", srr_))
        srr_ = 0.1;
    if(!private_nh_.getParam("srt", srt_))
        srt_ = 0.2;
    if(!private_nh_.getParam("str", str_))
        str_ = 0.1;
    if(!private_nh_.getParam("stt", stt_))
        stt_ = 0.2;
    if(!private_nh_.getParam("icp_thresh", icp_goodness_))
        icp_goodness_ = 0.8;

    // publisher 
    m_path_pub = m_node.advertise<nav_msgs::Path>("path", 1, true); 
    m_scan_pub = m_node.advertise<sensor_msgs::LaserScan>("raw_scan", 5);
    m_scan_pub2 = m_node.advertise<sensor_msgs::LaserScan>("re_raw_scan", 5);

    m_map_pub = m_node.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    m_meta_map_pub = m_node.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

    // build connection with publisher
    m_ack_pub = m_node.advertise<std_msgs::Bool>("ack", 1);
    m_syn_sub = m_node.subscribe("/syn", 1 , &LSODO_Listener::subsribeSYN, this);
    
    // for save path 
    m_path_save_pub = m_node.advertise<std_msgs::Bool>("save_path_ack",1); 
    m_path_save_sub = m_node.subscribe("/save_path_syn", 1, &LSODO_Listener::savePathSub, this);

    // laser receiver 
    m_scan_filter_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(m_node, "scan", 1); 
    m_scan_filter = new tf::MessageFilter<sensor_msgs::LaserScan>(*m_scan_filter_sub, m_tf, m_odom_frame, 5);
    m_scan_filter->registerCallback(boost::bind(&LSODO_Listener::laserCallback, this, _1));    

    // broadcast transform in another thread
    m_transform_thread = new boost::thread(boost::bind(&LSODO_Listener::publishLoop, this, transform_publish_period));

    // start to send data 
    // publishACK(true);
}

LSODO_Listener::~LSODO_Listener()
{
    if(m_transform_thread)
    {
        m_transform_thread->join(); 
        delete m_transform_thread; 
    }
    if(m_scan_filter_sub) 
        delete m_scan_filter_sub; 
    if(m_scan_filter)
        delete m_scan_filter ; 
    if(m_gsp_laser) 
        delete m_gsp_laser;
    if(m_gsp_odom)
        delete m_gsp_odom;
}

void LSODO_Listener::subsribeSYN(const std_msgs::BoolPtr& syn)
{
    cout<<"lsodo_lisntener.cpp: receive SYN !"<<endl;
    // cout<<"lsodo_lisntener.cpp: send ACK !"<<endl;
    publishACK();
}

void LSODO_Listener::savePathSub(const std_msgs::BoolPtr& syn)
{
    cout<<"lsodo_lisntener.cpp: receive save_path SYN !"<<endl; 
    static bool once = true; 
    if(once)
    {
        m_pGraphSlam->saveG2O();
        // m_pGraphSlam->savePath();
        m_pGraphSlam->optimizeGraph();
        m_pGraphSlam->savePathG2O("graph_g2o.txt");
        updateMap(scan_); 
        
        cout<<"lsodo.cpp: this is before iterative G2O"<<endl;
        // for(;;)
        {
            char k = int_info("lsodo.cpp: iterativeOptGraph() press 'e' or 'E' to exist! ");
            if(k == 'e' or k == 'E')
            {
                cout<<"lsodo.cpp: not use iterative g2o!"<<endl;
            }else
            {
                m_pGraphSlam->iterativeOptGraph();
            }
            m_pGraphSlam->savePathG2O("graph_my_g2o.txt");
            m_pGraphSlam->saveG2O("after_rebuild.g2o");
            updateMap(scan_); 
        }
        // m_pGraphSlam->savePathG2O("graph_my_g2o.txt");
        // m_pGraphSlam->saveG2O("after_rebuild.g2o");
        ROS_ERROR("lsodo.cpp: after iterative G2O");
        
        once = false;
    }
    std_msgs::BoolPtr ok(new std_msgs::Bool); 
    ok->data = true; 
    m_path_save_pub.publish(ok);
}

void LSODO_Listener::publishACK(bool delay)
{
    if(delay)
    {
        sleep(10);
        cout<<"lsodo_lisntener.cpp: sleep 1000ms before start!"<<endl;
    }
    // cout<<"lsodo_lisntener.cpp: send ACK"<<endl;
    std_msgs::BoolPtr ok(new std_msgs::Bool);
    ok->data = true;
    m_ack_pub.publish(ok);
}

void LSODO_Listener::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    m_laser_count++;

    // cout<<"lsodo_lisntener.cpp: receive "<<m_laser_count<<" item!"<<endl;
    if ((m_laser_count % m_throttle_scans) != 0)
    {
        publishACK();
        return;
    }
    static ros::Time last_map_update(0,0); 
    static bool s_first_scan = true;
    static bool s_handle_process = true;
    if(!s_handle_process)
    {
        publishACK();
        return ;
    }
    if(s_first_scan)
    {
        // m_laser_frame = scan->header.frame_id; 
        // cout<<"lsodo_listener.cpp: start to initMapper!"<<endl;
        D_COUT("lsodo_listener.cpp: start to initMapper!");
        if(! initMapper(*scan)) 
        {
            cout<<"failed to initMapper!"<<endl;
            return ;
        }
        s_first_scan = false;
        D_COUT("lsodo_listener.cpp finish initMapper!");
    }
    /*
    // debug to record laser
    ros::NodeHandle private_nh_("~");
    string save_path; 
    if(!private_nh_.getParam("save_path", save_path))
        save_path = "laser.txt"; 
    else
        save_path += "/laser.txt"; 
    static ofstream laser_record(save_path.c_str()); 
    laser_record<<scan->header.seq<<"\t"<<scan->header.stamp<<"\t"<<scan->header.frame_id<<endl; 
    laser_record<<scan->scan_time<<" "; 
    int num_ranges = scan->ranges.size();
       laser_record<<num_ranges<<" ";
    for(int i=0; i<num_ranges; i++)
        laser_record<<scan->ranges[i]<<" ";
    laser_record<<endl;
    */
    /*
    GMapping::OrientedPoint odom_pose; 
    
    if(getOdomPose(odom_pose, scan->header.stamp))
    {
        m_odom_traj.push_back(odom_pose);   
        m_odom_stamp.push_back(scan->header.stamp.sec + scan->header.stamp.nsec*1e-9);
    }else
    {
        cout<<"lsodo_listener.cpp: failed to getOdomPose!"<<endl;
    }
    */
    // cout<<"lsodo_listener.cpp: start to addScan!"<<endl;
    // SCAN alignment 
    GMapping::OrientedPoint odom_pose;
    if(addScan(*scan, odom_pose))
    {
        GMapping::OrientedPoint mpose = m_pGraphSlam->getCurrPose();
        cout<<"lsodo_listener.cpp: succeed to addScan, curr_pose: "<<&mpose<<endl;
        tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
        tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));
        // tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0.0));
        m_map_to_odom_mutex.lock();
        m_map_to_odom = (odom_to_laser * laser_to_map).inverse();
        m_map_to_odom_mutex.unlock();
        
        if(!m_got_map || (scan->header.stamp - last_map_update) > m_update_interval)
        {
            updateMap(*scan);
            last_map_update = scan->header.stamp;
            // m_pGraphSlam->saveG2O();
        }
        
        /*
        sensor_msgs::LaserScan::Ptr tmpScan(new sensor_msgs::LaserScan(*scan));
        // set this scan value using reverse Scan Reader
        CNodeScan* pcur = m_pGraphSlam->m_graph[m_pGraphSlam->m_graph.size()-1];
        vector<double > r; 
        pcur->reverseR(r);
        for(int i=0; i<r.size(); i++)
        {
            tmpScan->ranges[i] = r[i];
        }
        m_scan_pub2.publish(tmpScan);
        m_scan_pub.publish(scan);
        char k;
        cout<<"wait for input"<<endl;
        cin>>k;
        */
    }

    // std_msgs::BoolPtr ok(new std_msgs::Bool);
    // ok->data = true;
    // m_ack_pub.publish(ok);
    publishACK();

    /*if(m_pGraphSlam->size() > 100)
    {
        m_pGraphSlam->writePath("new_path.txt");
        s_handle_process = false;
        cout<<"lsodo_listener.cpp: after writePath stop handle process!"<<endl;
    }*/
/*
    // send odom_traj 
    nav_msgs::PathPtr path(new nav_msgs::Path);
    path->header.stamp = ros::Time::now();
    path->header.frame_id = m_base_frame; // m_map_frame;
    for(int i=0; i<m_odom_traj.size(); i++)
    {
        geometry_msgs::PoseStamped pose; 
        GMapping::OrientedPoint& p = m_odom_traj[i]; 
        pose.header.stamp.fromSec(m_odom_stamp[i]);
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y; 
        pose.pose.position.z = 0; 
        tf::Quaternion q; 
        q.setRPY(0, 0, p.theta);
        pose.pose.orientation.x = q.getX(); 
        pose.pose.orientation.y = q.getY();
        pose.pose.orientation.z = q.getZ();
        pose.pose.orientation.w = q.getW();

        path->poses.push_back(pose);
    }
    m_path_pub.publish(path);
 */
    
}

bool LSODO_Listener::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
    // static ofstream outf("odo_graph.txt"); 
    if(!getOdomPose(gmap_pose, scan.header.stamp))
    {
        return false;
    }
    // outf<<scan.header.stamp<<" : "<<&gmap_pose<<endl;

    double * ranges_double = new double[scan.ranges.size()];
    // If the angle increment is negative, we have to invert the order of the readings.
    if (m_gsp_laser_angle_increment < 0)
    {
        ROS_DEBUG("Inverting scan");
        int num_ranges = scan.ranges.size();
        for(int i=0; i < num_ranges; i++)
        {   
            // Must filter out short readings, because the mapper won't
            if(scan.ranges[i] < scan.range_min)
                ranges_double[i] = (double)scan.range_max;
            else
                ranges_double[i] = (double)scan.ranges[num_ranges - i - 1]; 
        }   
    } else 
    {
        for(unsigned int i=0; i < scan.ranges.size(); i++)
        {   
            // Must filter out short readings, because the mapper won't
            if(scan.ranges[i] < scan.range_min)
                ranges_double[i] = (double)scan.range_max;
            else
                ranges_double[i] = (double)scan.ranges[i];
        }   
    }
    // construct RangeReading -> node_scan 
    GMapping::RangeReading reading(scan.ranges.size(), 
                                    ranges_double, 
                                    m_gsp_laser,
                                    scan.header.stamp.toSec());
    delete []ranges_double;
    reading.setPose(gmap_pose);
    
    // cout<<"lsodo_listener.cpp: before addNode()"<<endl;
    D_COUT("lsodo_listener.cpp: before addNode()!");
    CNodeScan * new_node = new CNodeScan(reading);
    new_node->settimestamp(scan.header.stamp.toSec());

    if(m_pGraphSlam->addNode(new_node)) 
    {
        // cout<<"lsodo_listener.cpp: succeed addNode()"<<endl;
        D_COUT("lsodo_listener.cpp: succeed addNode()");
        return true; 
    }
    // cout<<"lsodo_listener.cpp: failed to addNode()"<<endl;
    delete new_node;
    return false;
}

bool LSODO_Listener::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
    // Get the laser's pose 
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                tf::Vector3(0,0,0)), t, m_laser_frame);
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        m_tf.transformPose(m_odom_frame, ident, odom_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());

    gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
            odom_pose.getOrigin().y(),
            yaw);
    return true;
}

void LSODO_Listener::publishLoop(double transform_publish_period)
{
    m_map_to_odom_mutex.lock();
    ros::Time tf_expiration = ros::Time::now() + ros::Duration(m_tf_delay);
    m_tfB->sendTransform( tf::StampedTransform (m_map_to_odom, tf_expiration, m_map_frame, m_odom_frame));
    m_map_to_odom_mutex.unlock();
}


bool LSODO_Listener::initMapper(const sensor_msgs::LaserScan& scan)
{
  m_laser_frame = scan.header.frame_id;
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = m_laser_frame;
  ident.stamp_ = scan.header.stamp;
  try
  {
    m_tf.transformPose(m_base_frame, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                                      m_base_frame);
  try
  {
    m_tf.transformPoint(m_laser_frame, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.z());
    return false;
  }

  m_gsp_laser_beam_count = scan.ranges.size();

  int orientationFactor;
  if (up.z() > 0)
  {
    orientationFactor = 1;
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    orientationFactor = -1;
    ROS_INFO("Laser is mounted upside down.");
  }

  m_angle_min = orientationFactor * scan.angle_min;
  m_angle_max = orientationFactor * scan.angle_max;
  m_gsp_laser_angle_increment = orientationFactor * scan.angle_increment;
  // ROS_DEBUG("Laser angles top down in laser-frame: min: %.3f max: %.3f inc: %.3f", angle_min_, angle_max_, gsp_laser_angle_increment_);

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // double maxRange_ , maxUrange_;
  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  m_gsp_laser = new GMapping::RangeSensor("FLASER",
                                         m_gsp_laser_beam_count,
                                         fabs(m_gsp_laser_angle_increment),
                                         gmap_pose,
                                         0.0,
                                         maxRange_);
  ROS_ASSERT(m_gsp_laser);

  GMapping::SensorMap smap;
  smap.insert(make_pair(m_gsp_laser->getName(), m_gsp_laser));
  // gsp_->setSensorMap(smap);

  m_pGraphSlam->setSensorMap(smap);

  m_gsp_odom = new GMapping::OdometrySensor(m_odom_frame);
  ROS_ASSERT(m_gsp_odom);

  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }
  
  // gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
  //                           kernelSize_, lstep_, astep_, iterations_,
  //                           lsigma_, ogain_, lskip_);
  m_pGraphSlam->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);
 
  // gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  m_pGraphSlam->setMotionModelParameters(srr_, srt_, str_, stt_);

  // gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  m_pGraphSlam->setlinearUpdate(linearUpdate_);
  m_pGraphSlam->setangularUpdate(angularUpdate_);
  ////  gsp_->setUpdatePeriod(temporalUpdate_);
  // gsp_->setgenerateMap(false);
  m_pGraphSlam->setGenerateMap(false);

  // gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
  //                               delta_, initialPose);
  m_pGraphSlam->init(xmin_, ymin_, xmax_, ymax_, m_delta_, initialPose);
  
  /* not used yet */
  // gsp_->setllsamplerange(llsamplerange_);
  // gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  // gsp_->setlasamplerange(lasamplerange_);
  // gsp_->setlasamplestep(lasamplestep_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1,time(NULL));
 
  // icp param 
  m_pGraphSlam->seticpGoodness(icp_goodness_);

  // copy this scan
  scan_ = scan;

  ROS_INFO("Initialization complete");
  return true;
}

void LSODO_Listener::publishMap(GMapping::ScanMatcher& matcher)
{
    if(!m_got_map) 
    {
        m_map_.map.info.resolution = m_delta_;
        m_map_.map.info.origin.position.x = 0.0;
        m_map_.map.info.origin.position.y = 0.0;
        m_map_.map.info.origin.position.z = 0.0;
        m_map_.map.info.origin.orientation.x = 0.0;
        m_map_.map.info.origin.orientation.y = 0.0;
        m_map_.map.info.origin.orientation.z = 0.0;
        m_map_.map.info.origin.orientation.w = 1.0;
    } 

    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, 
            m_delta_);
    ros::NodeHandle nh("~"); 
    bool show_key_node_only; 
    if(!nh.getParam("show_key_node_only", show_key_node_only))
        show_key_node_only = false;
    if(show_key_node_only)
    {
        m_pGraphSlam->getKeyNodeGridMap(matcher, &smap);
    }else
    {
        m_pGraphSlam->getGridMap(matcher, &smap);
    }
    /*
       ROS_DEBUG("Trajectory tree:");
       for(GMapping::GridSlamProcessor::TNode* n = best.node;
       n;
       n = n->parent)
       {
       ROS_DEBUG("  %.3f %.3f %.3f",
       n->pose.x,
       n->pose.y,
       n->pose.theta);
       if(!n->reading)
       {
       ROS_DEBUG("Reading is NULL");
       continue;
       }
       matcher.invalidateActiveArea();
       matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
       matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
       }
     */
    // the map may have expanded, so resize ros message as well
    if(m_map_.map.info.width != (unsigned int) smap.getMapSizeX() || m_map_.map.info.height != (unsigned int) smap.getMapSizeY()) 
    {
        // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
        //       so we must obtain the bounding box in a different way
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x; ymin_ = wmin.y;
        xmax_ = wmax.x; ymax_ = wmax.y;

        ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
                xmin_, ymin_, xmax_, ymax_);

        m_map_.map.info.width = smap.getMapSizeX();
        m_map_.map.info.height = smap.getMapSizeY();
        m_map_.map.info.origin.position.x = xmin_;
        m_map_.map.info.origin.position.y = ymin_;
        m_map_.map.data.resize(m_map_.map.info.width * m_map_.map.info.height);

        ROS_DEBUG("map origin: (%f, %f)", m_map_.map.info.origin.position.x, m_map_.map.info.origin.position.y);
    }
    for(int x=0; x < smap.getMapSizeX(); x++)
    {
        for(int y=0; y < smap.getMapSizeY(); y++)
        {
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            GMapping::IntPoint p(x, y);
            double occ=smap.cell(p);
            assert(occ <= 1.0);
            if(occ < 0)
                m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = -1;
            else if(occ > occ_thresh_)
            {
                //m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = (int)round(occ*100.0);
                m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = 100;
            }
            else
                m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = 0;
        }
    }
    // try only send active map not worked!
    /*
       typedef GMapping::HierarchicalArray2D<GMapping::PointAccumulator>::PointSet PPSET;
       const PPSET & activeArea = smap.storage().getActiveArea(); 
       for (PPSET::const_iterator it= activeArea.begin(); it!=activeArea.end(); it++)
       {
       int x = it->x; 
       int y = it->y;
       double occ=smap.cell(*it);
       assert(occ <= 1.0);
       if(occ < 0)
       m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = -1;
       else if(occ > occ_thresh_)
       {
    //m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = (int)round(occ*100.0);
    m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = 100;
    }
    else
    m_map_.map.data[MAP_IDX(m_map_.map.info.width, x, y)] = 0;
    } */  
    m_got_map = true;

    // make sure to set the header information on the map
    m_map_.map.header.stamp = ros::Time::now();
    m_map_.map.header.frame_id = m_tf.resolve( m_map_frame );

    // publish grid map
    m_map_pub.publish(m_map_.map);
    m_meta_map_pub.publish(m_map_.map.info);
}

void LSODO_Listener::publishPath()
{
    // publish path
    nav_msgs::PathPtr path ( new nav_msgs::Path);
    path->header.stamp = ros::Time::now();
    path->header.frame_id = m_map_frame;

    for(CGraph_SLAM::iterator it = m_pGraphSlam->begin();
            it != m_pGraphSlam->end();
            ++it)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp.fromSec(it->second->gettimestamp());//.fromSec(n->reading->getTime());
        GMapping::OrientedPoint* p = (it->second->getPose());
        pose.pose.position.x = p->x;;
        pose.pose.position.y = p->y;
        pose.pose.position.z = 0;
        tf::Quaternion q;
        q.setRPY(0,0,p->theta);
        pose.pose.orientation.x = q.getX();
        pose.pose.orientation.y = q.getY();
        pose.pose.orientation.z = q.getZ();
        pose.pose.orientation.w = q.getW();

        path->poses.push_back(pose);
    }
    // std::reverse(path->poses.begin(), path->poses.end());
    m_path_pub.publish(path);
}

void
LSODO_Listener::updateMap(const sensor_msgs::LaserScan& scan)
{
  boost::mutex::scoped_lock(m_map_mutex);
  GMapping::ScanMatcher matcher;
  double* laser_angles = new double[scan.ranges.size()];
  double theta = m_angle_min;
  for(unsigned int i=0; i<scan.ranges.size(); i++)
  {
    if (m_gsp_laser_angle_increment < 0)
        laser_angles[scan.ranges.size()-i-1]=theta;
    else
        laser_angles[i]=theta;
    theta += m_gsp_laser_angle_increment;
  }

  matcher.setLaserParameters(scan.ranges.size(), laser_angles,
                             m_gsp_laser->getPose());

  delete[] laser_angles;
  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  publishMap(matcher); 
  publishPath();
/*
  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);
*/

}


