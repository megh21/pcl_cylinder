#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

// void 
// cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
      {
          // Container for original & filtered data
          pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
          pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
          pcl::PCLPointCloud2 cloud_filtered;
          // std::cerr << "original" << std::endl;
          
          // Convert to PCL data type
          pcl_conversions::toPCL(*cloud_msg, *cloud);
          // std::cerr << "to PCL" << std::endl;
          
          // Perform the actual filtering
          pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
          sor.setInputCloud (cloudPtr);
          sor.setLeafSize (0.01, 0.01, 0.01);
          sor.filter (cloud_filtered);

          // Convert to ROS data type
          sensor_msgs::PointCloud2 output;
          pcl_conversions::moveFromPCL(cloud_filtered, output);

          // Publish the data
          pub.publish (output);
      }
int 
main (int argc, char** argv)
{   std::cerr << "1" << std::endl;
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  std::cerr << "2" << std::endl;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/zed2i/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
  std::cerr << "3" << std::endl;
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  std::cerr << "4" << std::endl;
  // Spin
  ros::spin ();

  return 0;
}