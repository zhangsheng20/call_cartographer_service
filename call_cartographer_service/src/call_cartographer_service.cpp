#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/ReadMetrics.h"

#include "ros/ros.h"

//rosrun call_cartographer_service call_cartographer_service 

int main(int argc,char **argv)
{
    ros::init(argc,argv,"call_carto_service");
    ros::NodeHandle myNodeHandle;
    int call_service_frequency=1;
    ros::ServiceClient client=myNodeHandle.serviceClient<cartographer_ros_msgs::ReadMetrics>("read_metrics");

    cartographer_ros_msgs::ReadMetrics srv;
    ros::Rate rate(call_service_frequency*10);
    while (myNodeHandle.ok())
    {
        if (client.call(srv))
        {
            //ROS_INFO("call ok!");

            for(int i=0;i < srv.response.metric_families.size();i++)
            {
                cartographer_ros_msgs::MetricFamily metricfamily_receive;
                metricfamily_receive=srv.response.metric_families[i];
                if(metricfamily_receive.name == "mapping_global_trajectory_builder_local_slam_results")
                {
                    {
                        int j=1;
                        double value=metricfamily_receive.metrics[j].value;
                        std::string headvalue= metricfamily_receive.metrics[j].labels[0].value;
                        ROS_INFO("%s: %f ",headvalue.c_str(),value);

                    }

                }

            

            }
    
        }
        else
        {
            ROS_ERROR("Failed to call service !");
        }
        rate.sleep();
    
    }
 

    return 0;

}