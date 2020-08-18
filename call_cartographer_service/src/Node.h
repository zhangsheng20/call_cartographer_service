
#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
class Node
{
private:
    ros::NodeHandle myNodeHandle;
    ros::ServiceClient start_trajectory_client;
    ros::ServiceClient finish_trajectory_client;
    ros::ServiceClient read_metrics_client;
    ros::ServiceClient get_trajectory_states_client;
    ros::ServiceClient trajectory_query_client;
    ros::Subscriber sub_move_base_simple;

    cartographer_ros_msgs::ReadMetrics read_metrics_srv;
    cartographer_ros_msgs::StartTrajectory start_trajectory_srv;
    cartographer_ros_msgs::FinishTrajectory finish_trajectory_srv;
    cartographer_ros_msgs::GetTrajectoryStates get_trajectory_states_srv;
    cartographer_ros_msgs::TrajectoryQuery trajectory_query_srv;


    void move_base_simple_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
public:
    Node();
    ~Node();

    void GetTrajectoryStates();
    void UpdateTrajectoryQuery();
    void StartTrajectory();
    void FinishTrajectory();
    void ReadMetrics();
    int GetCurrentTrajectoryId();
    void reinit_pose_from_RVIZ(geometry_msgs::Pose pose_from_rviz);
    void judge_SLAM_stare();
};

