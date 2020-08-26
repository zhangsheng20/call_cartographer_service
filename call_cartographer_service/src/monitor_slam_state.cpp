
#include "monitor_slam_state.h"
#include <math.h>

MonitorSlamState::MonitorSlamState()
{
    //this->time_begin_=ros::Time::now();
    this->trajectory_id_=0;
    this->mean_imu_to_ugv_.setRPY(0,0,0);
}

MonitorSlamState::MonitorSlamState(int trajectory_id ,ros::Time time_begin)
{
    this->time_begin_=time_begin;
    this->trajectory_id_=trajectory_id;
    this->mean_imu_to_ugv_.setRPY(0,0,0);
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
        //tf2::Quaternion imu_rotate;  //imu need to rotate 180 deg 
        //imu_rotate.setRPY(3.14,0,0);
        //imu_orientation_tf2=imu_rotate*imu_orientation_tf2;
        imu_to_ugv.setRotation(imu_orientation_tf2*ugv_orientation_tf2.inverse());   
        imu_to_ugv.stamp_=mavros_imu_data_.back().header.stamp;
        imu_to_ugv_.push_back(imu_to_ugv);
                  
        ROS_INFO("imu_Y:%f  ugv_Y:%f    ",GetYaw(imu_orientation_tf2) ,GetYaw(ugv_orientation_tf2));                     
        ROS_INFO("imu_to_ugv:%f",double(imu_to_ugv.getRotation().getAngle()));
        //imu_to_ugv*ugv=imu   than imu_to_ugv = imu *ugv^-1
        if(mean_imu_to_ugv_.getW()==1){
            CalcMeanImuToUgv();          
        }else
        {
            tf2::Quaternion imu_to_ugv_err_to_mean;
            imu_to_ugv_err_to_mean=imu_to_ugv*mean_imu_to_ugv_.inverse();
            ROS_INFO("slam_yaw_error:%f    ",GetYaw(imu_to_ugv_err_to_mean));   
            if((GetYaw(imu_to_ugv_err_to_mean))> 0.1 ||(GetYaw(imu_to_ugv_err_to_mean))<- 0.1){
                    ROS_INFO("!!!!!!!!!!!!!!!!!! SLAM ERROR !!!!!!!!!!!!!!  ");  
                } 
        }


    }
    else if(!mavros_imu_data_.empty() && ugv_odom_data_.empty() )
    {
        auto imu_orientation_msg = mavros_imu_data_.back().orientation;
        tf2::Quaternion imu_orientation_tf2;
        tf2::fromMsg(imu_orientation_msg,imu_orientation_tf2);
       
        //tf2::Quaternion imu_rotate;  //imu need to rotate 180 deg 
        //imu_rotate.setRPY(3.14,0,0);
        //imu_orientation_tf2=imu_rotate*imu_orientation_tf2;
       
        tf2Scalar imu_orientation_R,imu_orientation_P,imu_orientation_Y;
        tf2::Matrix3x3(imu_orientation_tf2).getRPY
                                (imu_orientation_R,imu_orientation_P,imu_orientation_Y);
                            
        ROS_INFO("imu_Y:%f   imu_R:%f ",imu_orientation_Y/3.1416*180 ,imu_orientation_R/3.1516*180);                     
        
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
    if(time_begin_.is_zero())
        time_begin_=msg->header.stamp;
       
    nav_msgs::Odometry input;
    input.child_frame_id=msg->child_frame_id;
    input.header=msg->header;
    input.pose=msg->pose;
    input.twist=msg->twist;
    ugv_odom_data_.push_back(input);
    //TrimUgvOdomData();

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
bool MonitorSlamState::CalcMeanImuToUgv()
{
    ros::Duration trajectory_duration =(imu_to_ugv_.back().stamp_-time_begin_);
    ROS_INFO("trajectory_duration: %f",trajectory_duration.toSec());
    if(trajectory_duration>(init_wait_duration+calc_mean_duration) )
    {

        double sum_x=0,sum_y=0,sum_z=0,sum_w=0 ;
        int cnt=0;
        for(auto imu_to_ugv:imu_to_ugv_)
        {
            if(imu_to_ugv_.back().stamp_- imu_to_ugv.stamp_<(calc_mean_duration))
            {
                sum_x+=imu_to_ugv.getRotation().getX();
                sum_y+=imu_to_ugv.getRotation().getY();
                sum_z+=imu_to_ugv.getRotation().getZ();
                sum_w+=imu_to_ugv.getRotation().getW();
                cnt++;
            }
        }
        mean_imu_to_ugv_.setX(sum_x/cnt);
        mean_imu_to_ugv_.setY(sum_y/cnt);
        mean_imu_to_ugv_.setZ(sum_z/cnt);
        mean_imu_to_ugv_.setW(sum_w/cnt);
        mean_imu_to_ugv_.normalize();
        return true;
    }
    return false;

}

tf2Scalar MonitorSlamState::GetYaw( tf2::Quaternion orientation_tf2)
{
    
    tf2Scalar orientation_R,orientation_P,orientation_Y;
    tf2::Matrix3x3(orientation_tf2).getRPY
                            (orientation_R,orientation_P,orientation_Y);
    return orientation_Y;

}

tf2Scalar MonitorSlamState::GetYaw( geometry_msgs::Quaternion orientation_msg)
{

    tf2::Quaternion orientation_tf2;
    tf2::fromMsg(orientation_msg,orientation_tf2);
    GetYaw(orientation_tf2);

}

tf2Scalar MonitorSlamState::GetPitch( tf2::Quaternion orientation_tf2)
{
    
    tf2Scalar orientation_R,orientation_P,orientation_Y;
    tf2::Matrix3x3(orientation_tf2).getRPY
                            (orientation_R,orientation_P,orientation_Y);
    return orientation_P;
}

tf2Scalar MonitorSlamState::GetRoll( tf2::Quaternion orientation_tf2)
{
    
    tf2Scalar orientation_R,orientation_P,orientation_Y;
    tf2::Matrix3x3(orientation_tf2).getRPY
                            (orientation_R,orientation_P,orientation_Y);
    return orientation_R;
}