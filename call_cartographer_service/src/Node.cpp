
#include "Node.h"


// subscribe callback function from Rviz

Node::Node()
{
    deque_duration_time.fromSec(10);
    judge_slam_state_period_sec.fromSec(0.3);

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

    ugv_odom_subscriber=myNodeHandle.subscribe<nav_msgs::Odometry>
                        ("UgvOdomTopic", 100,&Node::HandleUgvOdom,this);  

    mavros_imu_data_subscriber = myNodeHandle.subscribe<sensor_msgs::Imu>
                        ("/mavros/imu/data", 1000, &Node::HandleMavrosImuData,this);

    start_trajectory_srv.request.configuration_basename="my_rplidar_localization.lua";
    start_trajectory_srv.request.configuration_directory=
            "/home/ugv/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files";
    rviz_initialpose_subscriber = myNodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>
                        ("/initialpose", 1, &Node::HandleRvizInitialpose, this);


    judge_slam_state_timer_ = myNodeHandle.createTimer(judge_slam_state_period_sec,
                        &Node::JudgeSlamState, this);
    ROS_INFO("Init over!");
}

void Node::HandleRvizInitialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("get_rvizpose ok!");                            
    
    ROS_INFO("initial_pose orientation   w:%f x:%f y:%f z:%f",msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    ROS_INFO("initial_pose position    x:%f y:%f z:%f",msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,msg->pose.pose.position.z);  
    ReinitPoseFromRviz(msg->pose.pose);
    
}


void Node::ReinitPoseFromRviz(geometry_msgs::Pose pose_from_rviz)
{

    start_trajectory_srv.request.initial_pose=pose_from_rviz;
    start_trajectory_srv.request.relative_to_trajectory_id=0;
    start_trajectory_srv.request.use_initial_pose=true;

    finish_trajectory_srv.request.trajectory_id=GetCurrentTrajectoryId();

    FinishTrajectory();
    StartTrajectory();
    Mymonitor.SetNewTrajectory(GetCurrentTrajectoryId(),ros::Time(0));
    
}

void Node::JudgeSlamState(const ::ros::TimerEvent& timer_event)
{ 
    ROS_INFO("JudgeSlamState in!");
    Mymonitor.ComparePose();
    ROS_INFO("JudgeSlamState over! \n");
    //Mymonitor.CalcMeanImuToUgv();
    //ComparePose();
    //ReadMetrics();  
    //ShowMetricsFamily(12);
}

void Node::ShowMetricsFamily(int num)
{
    std::string name_switch;
    switch (num)
    {
    case 1:name_switch="mapping_constraints_constraint_builder_2d_constraints";break;
    case 2:name_switch="mapping_global_trajectory_builder_local_slam_results";break;
    case 3:name_switch="mapping_constraints_constraint_builder_2d_queue_length";break;
    case 4:name_switch="mapping_2d_local_trajectory_builder_latency";break;
    case 5:name_switch="mapping_2d_local_trajectory_builder_real_time_ratio";break;
    case 6:name_switch="mapping_2d_local_trajectory_builder_cpu_real_time_ratio";break;
    case 7:name_switch="mapping_2d_pose_graph_work_queue_delay";break;
    case 8:name_switch="mapping_2d_pose_graph_work_queue_size";break;
    case 9:name_switch="mapping_2d_pose_graph_constraints";break;
    case 10:name_switch="mapping_2d_pose_graph_submaps";break;
    case 11:name_switch="mapping_constraints_constraint_builder_2d_scores";break;
    case 12:name_switch="mapping_2d_local_trajectory_builder_scores";break;
    case 13:name_switch="mapping_2d_local_trajectory_builder_costs";break;
    case 14:name_switch="mapping_2d_local_trajectory_builder_residuals";break;
    default:
        break;
    }

    ROS_INFO("name: %s ",name_switch.c_str());
    for(auto& metricfamily_receive : read_metrics_srv.response.metric_families)
    {   
        if(metricfamily_receive.name == name_switch)
        {
            ROS_INFO("metrics: ");
            for(auto& metricfamily_receive_metric :metricfamily_receive.metrics)
            {
                
                double metric_value=metricfamily_receive_metric.value;
                
                if(!metricfamily_receive_metric.labels.empty())
                {
                    ROS_INFO("  labels: ");
                    for(auto label:metricfamily_receive_metric.labels)
                    {
                        std::string metric_label_key= label.key; 
                        std::string metric_label_value= label.value;
                        
                        ROS_INFO("    key:  %s ",metric_label_key.c_str());            
                        ROS_INFO("    value:%s ",metric_label_value.c_str());                                                                           
                    }

                }                
                ROS_INFO("  value:%f ",metric_value);
                if(!metricfamily_receive_metric.counts_by_bucket.empty()) // counts_by_bucket
                {
                    ROS_INFO("  counts_by_bucket: ");
                    for(auto& metric_counts_by_bucket : metricfamily_receive_metric.counts_by_bucket)
                    {
                        ROS_INFO("    bucket_boundary:%f  count:%f ",
                            metric_counts_by_bucket.bucket_boundary,metric_counts_by_bucket.count) ;
                                  
                    }
 
                }
                ROS_INFO("  ");
                
            }
            ROS_INFO("\n");
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
    read_metrics_client.call(read_metrics_srv);
    ROS_INFO("read_metrics response:%s",read_metrics_srv.response.status.message.c_str());
}

Node::~Node()
{
}


template <typename T>
void Node::TrimDequeData(std::deque<T>* data) 
{
    data->pop_front();
}

void Node::TrimMavrosImuData() 
{

    while(mavros_imu_data_.back().header.stamp-mavros_imu_data_.front().header.stamp
                                    >deque_duration_time)
    {
        TrimDequeData<sensor_msgs::Imu>(&mavros_imu_data_);
    }
}


void Node::TrimUgvOdomData()
{
    while(ugv_odom_data_.back().header.stamp-ugv_odom_data_.front().header.stamp
                                        >deque_duration_time)
    {
        TrimDequeData<nav_msgs::Odometry>(&ugv_odom_data_);
    }
    
}

void Node::HandleUgvOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
       
/*     nav_msgs::Odometry input;
    input.child_frame_id=msg->child_frame_id;
    input.header=msg->header;
    input.pose=msg->pose;
    input.twist=msg->twist;
    ugv_odom_data_.push_back(input);
    TrimUgvOdomData(); */
    Mymonitor.HandleUgvOdom(msg);

}
void Node::HandleMavrosImuData(const sensor_msgs::Imu::ConstPtr& msg)
{
/*     sensor_msgs::Imu input;
    input.header=msg->header;
    input.orientation=msg->orientation;
    input.angular_velocity=msg->angular_velocity;
    mavros_imu_data_.push_back(input);
    TrimMavrosImuData(); */
    Mymonitor.HandleMavrosImuData(msg);

}


 void Node::ComparePose()
 {
    if(!mavros_imu_data_.empty()&& !ugv_odom_data_.empty())
    {
        auto imu_orientation_msg = mavros_imu_data_.back().orientation;
        auto ugv_orientation_msg=ugv_odom_data_.back().pose.pose.orientation;  
        tf2::Quaternion imu_to_ugv;
        tf2::Quaternion imu_orientation_tf2;
        tf2::Quaternion ugv_orientation_tf2;
        tf2::fromMsg(imu_orientation_msg,imu_orientation_tf2);
        tf2::fromMsg(ugv_orientation_msg,ugv_orientation_tf2);
        imu_to_ugv=imu_orientation_tf2*ugv_orientation_tf2.inverse();   
        
        imu_to_ugv_.push_back(imu_to_ugv);
        ROS_INFO("imu_to_ugv:%f",double(imu_to_ugv.getAngle()));
        //imu_to_ugv*ugv=imu   than imu_to_ugv = imu *ugv^-1
    }
    
 }