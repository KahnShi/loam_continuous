#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

// new
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/ros/conversions.h>
// pcl fromROSMsg() has changed, need to include <pcl_conversions/pcl_conversions.h> header
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

class Converter{
public:
  Converter();
  ros::NodeHandle nh;
  ros::Subscriber subLaserScan;
  ros::Publisher pubLaserCloud;
  ros::Subscriber subOdom;
  ros::Publisher pubImu;
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
  void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_in);
};

Converter::Converter(){
  subLaserScan = nh.subscribe<sensor_msgs::LaserScan> 
    ("/scan", 2, &Converter::scanCallback, this);
  
  subOdom = nh.subscribe<nav_msgs::Odometry> 
    ("/dji_sdk/odometry", 2, &Converter::odomCallback, this);
  
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> 
    ("/sync_scan_cloud_filtered", 2);
  
  pubImu = nh.advertise<sensor_msgs::Imu> 
    ("/microstrain/imu", 2);

}

void Converter::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){

  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud2 cloud;
  projector_.projectLaser(*scan_in, cloud);
  cloud.header.frame_id = "/camera";
  pubLaserCloud.publish(cloud);
}


void Converter::odomCallback (const nav_msgs::Odometry::ConstPtr& odom_in){
  sensor_msgs::Imu imu;
  imu.header = odom_in->header;
  imu.header.frame_id = "microstrain";
  imu.orientation = odom_in->pose.pose.orientation;
  // imu.orientation_covariance = odom_in->pose.covariance;
  imu.angular_velocity = odom_in->twist.twist.angular;
  pubImu.publish(imu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanToPt2");

  Converter cvt;
  
  ros::spin();

  return 0;
}
