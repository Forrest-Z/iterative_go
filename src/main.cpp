#include "ros/ros.h"

#include "lsodo_listener.h"
#include "graph_slam.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lsodo_listener"); 
    ros::NodeHandle pr("~"); 
    bool test = false; 
    pr.param("only_test", test, false); 
    if(test)
    {
        string fname("before_rebuild.g2o");
        if(argc >=2 )
            fname = string(argv[1]);
        testOptimizer(fname.c_str());
        return 0;
    }
   
    LSODO_Listener lsodo_listener; 
    ros::spin();
    return 0;
}
