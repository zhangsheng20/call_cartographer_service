#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include "Node.h"


// subscribe callback function from Rviz

Node::Node()
{

    start_trajectory_client=myNodeHandle.serviceClient
                            <cartographer_ros_msgs::StartTrajectory>("start_trajectory");
    finish_trajectory_client=myNodeHandle.serviceClient
                            <cartographer_ros_msgs::FinishTrajectory>("finish_trajectory");
    read_metrics_client=myNodeHandle.serviceClient
                            <cartographer_ros_msgs::ReadMetrics>("read_metrics");
    get_trajectory_states_client=myNodeHandle.serviceClient
                            <cartographer_ros_msgs::GetTrajectoryStates>("get_trajectory_states");
    trajectory_query_client=myNodeHandle.serviceClient
                            <cartographer_ros_msgs::TrajectoryQuery>("trajectory_query");


    start_trajectory_srv.request.configuration_basename="my_rplidar_localization.lua";
    start_trajectory_srv.request.configuration_directory=
            "/home/ugv/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files";
    sub_move_base_simple = myNodeHandle.subscribe  
                <geometry_msgs::PoseWithCovarianceStamped>
                        ("/initialpose", 1, &Node::move_base_simple_callback, this); 
    
    ROS_INFO("Init over!");
}

void Node::move_base_simple_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("get_rvizpose ok!");                            
    
    ROS_INFO("initial_pose orientation   w:%f x:%f y:%f z:%f",msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    ROS_INFO("initial_pose position    x:%f y:%f z:%f",msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,msg->pose.pose.position.z);  
    reinit_pose_from_RVIZ(msg->pose.pose);

}


void Node::reinit_pose_from_RVIZ(geometry_msgs::Pose pose_from_rviz)
{

    start_trajectory_srv.request.initial_pose=pose_from_rviz;
    start_trajectory_srv.request.relative_to_trajectory_id=0;
    start_trajectory_srv.request.use_initial_pose=true;

    finish_trajectory_srv.request.trajectory_id=GetCurrentTrajectoryId();

    FinishTrajectory();
    StartTrajectory();
}

void Node::judge_SLAM_stare()
{ 
    ROS_INFO("judge_SLAM_stare in!");
    ReadMetrics();

    for(auto& metricfamily_receive : read_metrics_srv.response.metric_families)
    {                      
        for(auto& metricfamily_receive_metric :metricfamily_receive.metrics)
        {
            double metric_value=metricfamily_receive_metric.value;
                
            if(!metricfamily_receive_metric.labels.empty())
            {
                std::string metric_label_value= metricfamily_receive_metric.labels[0].value;
                std::string metric_label_key= metricfamily_receive_metric.labels[0].key;
                if(metricfamily_receive.name == "mapping_global_trajectory_builder_local_slam_results")
                {                     
                    //ROS_INFO("%s: %f ",metric_label_value.c_str(),metric_value);  
                }
                else if(metricfamily_receive.name =="mapping_constraints_constraint_builder_2d_scores") 
                {
                    ROS_INFO("metric_label_key:%s  metric_label_value:%s ",
                                metric_label_key.c_str(),metric_label_value.c_str());
                    for(auto& metric_counts_by_bucket : metricfamily_receive_metric.counts_by_bucket)
                    {
                        ROS_INFO("    bucket_boundary:%f  count:%f ",
                            metric_counts_by_bucket.bucket_boundary,metric_counts_by_bucket.count) ;
                    }
                    
                }
            }

        }

    }

    
 
}

void Node::GetTrajectoryStates()
{
    if(get_trajectory_states_client.call(get_trajectory_states_srv))
    {
        ROS_INFO("get_trajectory_states ok!");
    }
}

void Node::UpdateTrajectoryQuery()
{
    GetTrajectoryStates();
    trajectory_query_srv.request.trajectory_id=GetCurrentTrajectoryId();
    if(trajectory_query_client.call(trajectory_query_srv))
    {
        ROS_INFO("trajectory_query ok!");
    }

}

int Node::GetCurrentTrajectoryId()
{
    GetTrajectoryStates();
    return get_trajectory_states_srv.response.trajectory_states.trajectory_id.back();
}


void Node::StartTrajectory()
{
    if( start_trajectory_client.call(start_trajectory_srv))
    {
        ROS_INFO("start_trajectory ok! new trajectory_id:%d",start_trajectory_srv.response.trajectory_id);
    }
}
void Node::FinishTrajectory()
{
    if(finish_trajectory_client.call(finish_trajectory_srv))
    {
        ROS_INFO("finish_trajectory ok! status.code:%d",finish_trajectory_srv.response.status.code);
    }

}

void Node::ReadMetrics()
{
    if (read_metrics_client.call(read_metrics_srv))
    {
        ROS_INFO("read_metrics ok!");
    }
}

Node::~Node()
{
}
