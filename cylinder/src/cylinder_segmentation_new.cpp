//#include "ros/ros.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/sample_consensus/sac_model_circle2d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include "std_msgs/String.h"
// #include <sstream>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>


typedef pcl::PointXYZ PointT;
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
    seg.setDistanceThreshold(0.01);
    seg.setRadiusLimits(0.1, 0.5);

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

int main ()
{  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  //ros::init(argc, argv, "listener");
  //ros::NodeHandle n;
  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  //ros::Subscriber sub = n.subscribe("chatter", 10, *cloud);
  reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  //std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;
  // Build a passthrough filter to remove spurious NaNs and scene background
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  //Create the segmentation object for the CIRCLE2D and set all the parameters
  /*
  seg.setModelType (pcl::SACMODEL_CIRCLE2D);
  */
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);
  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
  writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);
  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);
  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
	  writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  }

  // Create a SAC model for circle fitting

  // Read in the point cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud2);

  // sac model
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg1;
  seg1.setOptimizeCoefficients (true);
  seg1.setModelType(pcl::SACMODEL_CIRCLE2D);
  seg1.setMethodType(pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::PointIndices::Ptr inlierss (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficientss (new pcl::ModelCoefficients);
  seg1.setInputCloud(cloud2);

  // Fit a circle to the point cloud
   seg1.segment (*inlierss, *coefficientss);
  
  std::cout << "Circle coefficients: " << coefficientss << std::endl;
  std::cout << "Number of inliers: " << inlierss->indices.size() << std::endl;


if (inlierss->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a circle model for the given dataset.");
        return (-1);
    }

    // Create the PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("3D Circle Detection");
    viewer.setBackgroundColor (0, 0, 0);

    // Add the point cloud to the viewer
    viewer.addPointCloud (cloud2);

    // Create a new point cloud to hold the inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud (*cloud2, inlierss->indices, *circle_cloud);

    // Add the inliers to the viewer as a red cloud
    viewer.addPointCloud (circle_cloud, "circle_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "circle_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "circle_cloud");
    
    // Create a new ModelCoefficients object
    pcl::ModelCoefficients circle_coeff;
    circle_coeff.values.resize(7);
    // Fill the coefficients of the circle
    for (size_t i = 0; i < 7; i++)
    {
        circle_coeff.values[i] = coefficientss->values[i];
    }
    // Add the circle to the viewer
    viewer.addCircle(circle_coeff, "circle");

    // Run the viewer
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

/**************************************************************************/

  // // pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
  // // viewer.showCloud(cloud_cylinder);
  // // while (!viewer.wasStopped()) {}
  
  // Create a PCLVisualizer object

  // pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
  // // Add the point cloud to the viewer
  // viewer.addPointCloud(cloud_cylinder, "cloud cylinder");
  // //viewer.addPointCloud(cloud, "cloud");
  // // Set the background color of the viewer
  // viewer.setBackgroundColor(0, 0, 0);
  // // Set the size of the points in the point cloud
  // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);


  // // Run the viewer
  // while (!viewer.wasStopped())
  // {
  //   viewer.spinOnce();
  // }


  return (0);
}





// read from ros pointcloud2 to pcl
/*
 void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //do stuff with temp_cloud here
    }
*/


/*
** For Debug only *****
*
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int ros_listen(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);

  
  ros::spin();

  return 0;
}
*/
