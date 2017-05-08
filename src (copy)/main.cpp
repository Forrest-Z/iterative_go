#include "ros/ros.h"

#include "lsodo_listener.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lsodo_listener"); 
    LSODO_Listener lsodo_listener; 
    ros::spin();
    return 0;
}
