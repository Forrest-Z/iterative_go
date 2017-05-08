#ifndef LSODO_LISTENER_H
#define LSODO_LISTENER_H

/*Author: David_Z Jan 17 2014*/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "std_msgs/Bool.h"

#include <vector>
#include <boost/thread.hpp>

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

using namespace std;

class CGraph_SLAM;

class LSODO_Listener
{
public:
    LSODO_Listener();
    ~LSODO_Listener();
    
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void publishLoop(double transform_publish_period);
protected:
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
    bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);
    bool initMapper(const sensor_msgs::LaserScan& scan);
    void updateMap(const sensor_msgs::LaserScan& scan);
private:
    // publishers
    ros::NodeHandle m_node;
    ros::Publisher m_path_pub; 
    ros::Publisher m_scan_pub;
    ros::Publisher m_map_pub;
    ros::Publisher m_meta_map_pub;

    void publishACK(bool delay = false);
    void subsribeSYN(const std_msgs::BoolPtr&);
    ros::Publisher m_ack_pub; // to syn handling scan
    ros::Subscriber m_syn_sub;
    
    // TODO: for comparison !!
    // GMapping::GridSlamProcessor* gsp_;
    
    // tf + msgs 
    message_filters::Subscriber<sensor_msgs::LaserScan>* m_scan_filter_sub; 
    tf::MessageFilter<sensor_msgs::LaserScan>* m_scan_filter; 
    tf::TransformBroadcaster * m_tfB;
    tf::TransformListener m_tf;
    tf::Transform m_map_to_odom; 
    int m_laser_count;
    int m_throttle_scans;
    double m_tf_delay;
    
    // graph-slam algorithm
    CGraph_SLAM * m_pGraphSlam;
    bool m_gotFirstScan ;
    
    // updateMap interval
    ros::Duration m_update_interval;

    // laser parameters
    unsigned int m_gsp_laser_beam_count;
    double m_gsp_laser_angle_increment;
    double m_angle_min; 
    double m_angle_max;
    double maxRange_;
    double maxUrange_;
    
    // map range 
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;

    double occ_thresh_;    // occlude threshold for grid map
    double sigma_;      // sigma for laser observation for score
    int kernelSize_;  // window size for searching matched point

    // climb-hill parameters
    double lstep_;    // climb-hill guess initial search step (linear)
    double astep_;    // climb-hill guess initial search step (angular)
    int iterations_;  // number of refinement steps in the scan matching.  
                      // The final "precision" for the match is lstep*2^(-iterations) astep*2^(-iterations), respectively.
    
    // smaller step in a smaller range, not use
    double llsamplerange_; 
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;

    double lsigma_;   // sigma for laser observation for likelihood
    double ogain_;    // smoothing likelihood
    int lskip_;       // skip part of the beams to calculate likelihood
    
    // icp threshold 
    double icp_goodness_;

    // update distance 
    double linearUpdate_;
    double angularUpdate_;

    // motion parameters
    double srr_;
    double srt_;
    double str_;
    double stt_;

    // map parameter 
    bool m_got_map;
    nav_msgs::GetMap::Response m_map_;
    GMapping::RangeSensor* m_gsp_laser;
    GMapping::OdometrySensor* m_gsp_odom;
    double m_delta_;

    // tf frames
    std::string m_base_frame;
    std::string m_laser_frame;
    std::string m_map_frame; 
    std::string m_odom_frame;

    boost::thread* m_transform_thread;
    boost::mutex m_map_to_odom_mutex; 
    boost::mutex m_map_mutex;

    vector<GMapping::OrientedPoint> m_odom_traj;
    vector<double> m_odom_stamp;
};


#endif
