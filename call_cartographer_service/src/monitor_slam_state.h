#include "ros/ros.h"

#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deque>


class MonitorSlamState
{
private:
    int trajectory_id_;
    ros::Time time_begin_;
    tf2::Quaternion mean_imu_to_ugv_;
    ros::Duration init_wait_duration=ros::Duration(3);

    ros::Duration deque_duration_time=ros::Duration(10);
    std::deque<sensor_msgs::Imu> mavros_imu_data_;
    std::deque<nav_msgs::Odometry> ugv_odom_data_;
    std::deque<tf2::Stamped<tf2::Transform>> imu_to_ugv_;
public:
    MonitorSlamState();
    MonitorSlamState(int trajectory_id ,ros::Time time_begin);
    ~MonitorSlamState();
  
    void HandleMavrosImuData(const sensor_msgs::Imu::ConstPtr& msg);
    void HandleUgvOdom(const nav_msgs::Odometry::ConstPtr& msg);
    template <typename T> void TrimDequeData(std::deque<T>* data);
    void TrimMavrosImuData();
    void TrimUgvOdomData();
    void ComparePose();
    void SetNewTrajectory(int trajectory_id,ros::Time time_begin);
    void CalcMeanImuToUgv();
};
