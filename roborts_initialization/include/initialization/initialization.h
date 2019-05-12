#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/registration/icp.h"

#include "ros/ros.h"
#include <Eigen/Dense>


#define CREATE_LASER_CLOUD
//#define CREATE_MAP_CLOUD
//#define CREATE_MATCH_CLOUD


using namespace std;
struct Point2
{
    float x;
    float y;

};

struct match2
{
    float x;
    float y;
    float yaw;
    float score;
};

class initialization
{

    ros::NodeHandle nodeHandle;

    ros::Subscriber laserSubscriber;
    ros::Subscriber mapSubscriber;
    //uwb
    ros::Subscriber uwbSubscriber;

    ros::Publisher posePublisher;
    tf::TransformListener transformListener;
    tf::TransformListener tf_listener_ptr_;


    ros::Publisher marker_pub; 

    nav_msgs::OccupancyGrid map;

    #ifdef CREATE_LASER_CLOUD
    pcl::PointCloud<pcl::PointXYZ> laserCloud;
    ros::Publisher pointCloudPublisher;
    #endif

    #ifdef CREATE_MAP_CLOUD
    pcl::PointCloud<pcl::PointXYZ> mapPointCloud;
    ros::Publisher mapPublisher;
    #endif

    #ifdef CREATE_MATCH_CLOUD
    pcl::PointCloud<pcl::PointXYZ> matchCloud;
    ros::Publisher matchPublisher;
    #endif

    pcl::PointCloud<pcl::PointXYZ> artLaserCloud;
    ros::Publisher artLaserPublisher;

    vector<char> map_data;
    int map_width, map_height;
    float map_resolution, map_position_x, map_position_y;
    double laser_x, laser_y, laser_yaw;

    sensor_msgs::LaserScan scan_in;

    bool we_have_map;
  
    //uwb
    bool enable_uwb_ = true;
    bool uwb_init_ = true;
    bool update_uwb_ = true;
    int uwb_thread_delay_ = 10;
    std::string uwb_frame_;
    std::string uwb_topic_name_;
    ros::Time uwb_latest_time;
    Eigen::Vector3d uwb_latest_pose_;

    vector<Point2> sensor_data, sensor_data_reduced;


public:

    initialization();

    void callbackLaser(const sensor_msgs::LaserScan::ConstPtr&);
    void callbackMap(nav_msgs::OccupancyGrid);
    void UwbCallback(const geometry_msgs::PoseStamped::ConstPtr &uwb_msg);
    void pub();
    float calculateMatch(vector<Point2>& , float);

    void applyICPTransform();


    vector<match2> basitAlignment(float);

    vector<Point2> artificialSensorData(float, float, float);
    float compareArtificialAndRealSensorData(float, float, float);
    bool compareUwbMatch(float, float);
    void artMatch();

    void publishPose(float, float, float);


};