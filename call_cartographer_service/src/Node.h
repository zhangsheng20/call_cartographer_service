
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

#include <deque>


class Node
{
private:
    ros::ServiceClient start_trajectory_client;
    ros::ServiceClient finish_trajectory_client;
    ros::ServiceClient read_metrics_client;
    ros::ServiceClient get_trajectory_states_client;
    ros::ServiceClient trajectory_query_client;

    ros::Subscriber rviz_initialpose_subscriber;
    ros::Subscriber laser_odom_subscriber;
    ros::Subscriber ugv_odom_subscriber;
    ros::Subscriber mavros_imu_data_subscriber;

    cartographer_ros_msgs::ReadMetrics read_metrics_srv;
    cartographer_ros_msgs::StartTrajectory start_trajectory_srv;
    cartographer_ros_msgs::FinishTrajectory finish_trajectory_srv;
    cartographer_ros_msgs::GetTrajectoryStates get_trajectory_states_srv;
    cartographer_ros_msgs::TrajectoryQuery trajectory_query_srv;

    ros::Duration deque_duration_time=ros::Duration(10);
    std::deque<sensor_msgs::Imu> mavros_imu_data_;
    std::deque<nav_msgs::Odometry> ugv_odom_data_;

    ros::Duration judge_slam_state_period_sec=ros::Duration(0.3);
    ros::Timer judge_slam_state_timer_;


    void HandleRvizInitialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void HandleUgvOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void HandleMavrosImuData(const sensor_msgs::Imu::ConstPtr& msg);

    template <typename T> void TrimDequeData(std::deque<T>* data);
    void TrimMavrosImuData();
    void TrimUgvOdomData();

    public:
    Node();
    ~Node();
    ros::NodeHandle myNodeHandle;
    void GetTrajectoryStates();
    void UpdateTrajectoryQuery();
    void StartTrajectory();
    void FinishTrajectory();
    void ReadMetrics();

    void ShowMetricsFamily(int num);
    int GetCurrentTrajectoryId();
    void ReinitPoseFromRviz(geometry_msgs::Pose pose_from_rviz);
    void JudgeSlamState(const ::ros::TimerEvent& timer_event);
};

