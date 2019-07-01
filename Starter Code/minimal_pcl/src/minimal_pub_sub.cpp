#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
  // Create container variables
  pcl::PCLPointCloud2 cloud;

  // Publish the data
  pub.publish (cloud);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ros_pcl_minimal_pub_sub");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("test_output", 1);

  // Spin
  ros::spin ();
}
