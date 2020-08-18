#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include "Node.h"



//rosrun 


int main(int argc,char **argv)
{
    ros::init(argc,argv,"call_carto_service");
    
    int call_service_frequency=1;
    Node mycall;   

    ros::spin();
    ROS_INFO("ROS over!");

    return 0;
    

}

