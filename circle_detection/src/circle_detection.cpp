#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert the ROS message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Create a segmentation object for circle detection
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.001);
    seg.setRadiusLimits(0.01, 0.5);

    // Create a point inliers indices and model coefficients
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Segment the point cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Extract the inliers from the point cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);

    // Convert the PCL point cloud back to a ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = msg->header;

    // Publish the output point cloud
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_detection");
    ros::NodeHandle nh;

    // Subscribe to the point cloud topic
    ros::Subscriber sub = nh.subscribe("/zed2i/zed_node/point_cloud/cloud_registered", 1, pointcloudCallback);

     // Advertise the output point cloud topic
    pub = nh.advertise<sensor_msgs::PointCloud2>("detected", 1);

    ros::spin();

    return 0;
}
