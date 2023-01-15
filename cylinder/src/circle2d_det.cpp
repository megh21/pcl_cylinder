#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
  // Read in the point cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

  // Create the segmentation object for circle detection
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a circle model for the given dataset.");
    return (-1);
  }

  // Print the circle coefficients
  std::cout << "Model coefficients: " << *coefficients << std::endl;

    double min_radius = 0.1;
    double max_radius = 1.0;

  double radius = sqrt(coefficients->values[3]*coefficients->values[3] +
    coefficients->values[4]*coefficients->values[4] +
    coefficients->values[5]*coefficients->values[5] +
    coefficients->values[6]);
  if (radius < min_radius || radius > max_radius) {
    PCL_ERROR("Circle radius is out of bounds. Skipping visualization.");
    return -1;
  }


    // Create the PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("2D Circle Detection");
    viewer.setBackgroundColor (0, 0, 0);

    // Add the point cloud to the viewer
    viewer.addPointCloud (cloud);

    // Create a new point cloud to hold the inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud (*cloud, inliers->indices, *circle_cloud);

    // Add the inliers to the viewer as a red cloud
    viewer.addPointCloud (circle_cloud, "circle_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "circle_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "circle_cloud");
    
    // Create a new ModelCoefficients object
    pcl::ModelCoefficients circle_coeff;
    circle_coeff.values.resize(sizeof(coefficients)/sizeof(*coefficients));
    // Fill the coefficients of the circle
    for (size_t i = 0; i < sizeof(coefficients)/sizeof(*coefficients); i++)
    {
        circle_coeff.values[i] = coefficients->values[i];
    }
    // Add the circle to the viewer
    viewer.addCircle(circle_coeff, "circle");

    // Run the viewer
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

  return (0);


}

 // double x = coefficients->values[0];
    // double y = coefficients->values[1];
    // double z = coefficients->values[2];
    // double r = sqrt(coefficients->values[3]*coefficients->values[3] +
    //     coefficients->values[4]*coefficients->values[4] +
    //     coefficients->values[5]*coefficients->values[5] +
    //     coefficients->values[6]);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // for (int i = 0; i < 360; i++)
    // {
    //     double theta = (i / 180.0) * M_PI;
    //     double x_ = x + r*cos(theta);
    //     double y_ = y + r*sin(theta);
    //     double z_ = z;

    //     pcl::PointXYZ point;
    //     point.x = x_;
    //     point.y = y_;
    //     point.z = z_;
    //     circle_cloud->points.push_back(point);
    // }
    // circle_cloud->width = circle_cloud->points.size();
    // circle_cloud->height = 1; 

    // pcl::visualization::CloudViewer viewer("3D Circle Detection");
    // viewer.showCloud(circle_cloud);
    // while (!viewer.wasStopped()) {}


