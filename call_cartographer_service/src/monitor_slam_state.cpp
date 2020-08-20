
#include "monitor_slam_state.h"


MonitorSlamState::MonitorSlamState()
{
    this->time_begin_=ros::Time::now();
    this->trajectory_id_=0;
}

MonitorSlamState::MonitorSlamState(int trajectory_id ,ros::Time time_begin)
{
    this->time_begin_=time_begin;
    this->trajectory_id_=trajectory_id;


}

MonitorSlamState::~MonitorSlamState()
{
}

 void MonitorSlamState::ComparePose()
 {
    if(!mavros_imu_data_.empty()&& !ugv_odom_data_.empty())
    {
        auto imu_orientation_msg = mavros_imu_data_.back().orientation;
        auto ugv_orientation_msg=ugv_odom_data_.back().pose.pose.orientation;  
        tf2::Stamped<tf2::Transform> imu_to_ugv;
        tf2::Quaternion imu_orientation_tf2;
        tf2::Quaternion ugv_orientation_tf2;
        tf2::fromMsg(imu_orientation_msg,imu_orientation_tf2);
        tf2::fromMsg(ugv_orientation_msg,ugv_orientation_tf2);
        imu_to_ugv.setRotation(imu_orientation_tf2*ugv_orientation_tf2.inverse());   
        imu_to_ugv.stamp_=mavros_imu_data_.back().header.stamp;
        imu_to_ugv_.push_back(imu_to_ugv);
        double imu_orientation_R,imu_orientation_P,imu_orientation_Y;
        tf2::Matrix3x3(imu_orientation_tf2).getRPY
                                (imu_orientation_R,imu_orientation_P,imu_orientation_Y);
        ROS_INFO("imu_orientation_Y:%f",imu_orientation_Y);                     
        ROS_INFO("imu_to_ugv:%f",double(imu_to_ugv.getRotation().getAngleShortestPath()));
        //imu_to_ugv*ugv=imu   than imu_to_ugv = imu *ugv^-1
    }
 }


 template <typename T>
void MonitorSlamState::TrimDequeData(std::deque<T>* data) 
{
    data->pop_front();
}

void MonitorSlamState::TrimMavrosImuData() 
{

    while(mavros_imu_data_.back().header.stamp-mavros_imu_data_.front().header.stamp
                                    >deque_duration_time)
    {
        TrimDequeData<sensor_msgs::Imu>(&mavros_imu_data_);
    }
}


void MonitorSlamState::TrimUgvOdomData()
{
    while(ugv_odom_data_.back().header.stamp-ugv_odom_data_.front().header.stamp
                                        >deque_duration_time)
    {
        TrimDequeData<nav_msgs::Odometry>(&ugv_odom_data_);
    }
    
}

void MonitorSlamState::HandleUgvOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
       
    nav_msgs::Odometry input;
    input.child_frame_id=msg->child_frame_id;
    input.header=msg->header;
    input.pose=msg->pose;
    input.twist=msg->twist;
    ugv_odom_data_.push_back(input);
    TrimUgvOdomData();

}
void MonitorSlamState::HandleMavrosImuData(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu input;
    input.header=msg->header;
    input.orientation=msg->orientation;
    input.angular_velocity=msg->angular_velocity;
    mavros_imu_data_.push_back(input);
    TrimMavrosImuData();

}

void MonitorSlamState::SetNewTrajectory(int trajectory_id,ros::Time time_begin)
{
    this->time_begin_=time_begin;
    this->trajectory_id_=trajectory_id;
    mavros_imu_data_.clear();
    ugv_odom_data_.clear();
    imu_to_ugv_.clear();  
    mean_imu_to_ugv_.setRPY(0,0,0);
}
void MonitorSlamState::CalcMeanImuToUgv()
{
    ros::Duration calc_data_duration=ros::Duration(3);
    if(imu_to_ugv_.back().stamp_-time_begin_>(init_wait_duration+calc_data_duration))
    {
        double sum=0 ;
        for(auto imu_to_ugv:imu_to_ugv_)
        {
            sum+=imu_to_ugv.getRotation().getAngleShortestPath();




        }
    }


}