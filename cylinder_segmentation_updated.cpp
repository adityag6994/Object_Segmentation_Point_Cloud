#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <pcl/common/centroid.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::ModelCoefficients::ConstPtr coefficients_cylinder)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 1);
  //void pcl::visualization::PCLVisualizer::setBackgroundColor  (   const double &    r, 
  //bool pcl::visualization::PCLVisualizer::addSphere   (   const PointT &    center, 
  // viewer->addSphere<pcl::PointXYZ> ()
  //viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.3);
  viewer->initCameraParameters ();
  viewer->addLine<pcl::PointXYZ> (cloud->points[0], cloud->points[1], "line");
  
  viewer->addSphere (cloud->points[0], 0.01, "sphere");
  viewer->addSphere (cloud->points[2], 0.02, "sphere1");
  //viewer->addSphere (cloud->points[0], 0.01, "sphere");

  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (coefficients_cylinder->values[0]);
  coeffs.values.push_back (coefficients_cylinder->values[1]);
  coeffs.values.push_back (coefficients_cylinder->values[2]);
  coeffs.values.push_back (coefficients_cylinder->values[3]);
  coeffs.values.push_back (coefficients_cylinder->values[4]);
  coeffs.values.push_back (coefficients_cylinder->values[5]);
  coeffs.values.push_back (coefficients_cylinder->values[6]);
  viewer->addCylinder (coeffs, "cylinder");

  //return (viewer);
  
  return (viewer);
}


typedef pcl::PointXYZ PointT;


int
main (int argc, char** argv)
{
  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  //reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  reader.read ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud);
  std::cerr << " -------------------------------------------------------------" << std::endl;
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
  

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    writer.write ("test_cylinder.pcd", *cloud_cylinder, false);
  }

  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for (size_t i = 0; i < cloud->points.size(); ++i){
       centroid.add(cloud->points[i]);
  }
  pcl::PointXYZ c1;
  centroid.get (c1);
  std::cout << "||" <<c1.x << "||" << c1.y << "||" << c1.z << std::endl;
  
  pcl::PointCloud<PointT>::Ptr cloud_test (new pcl::PointCloud<PointT> ());
  *cloud_test = *cloud;
  cloud_test->points[0].x =  coefficients_cylinder->values[0]; 
  cloud_test->points[0].y =  coefficients_cylinder->values[1]; 
  cloud_test->points[0].z =  coefficients_cylinder->values[2]; 
  
  cloud_test->points[1].x =  cloud->points[cloud->size() - 1].x; 
  cloud_test->points[1].y =  cloud->points[cloud->size() - 1].y; 
  cloud_test->points[1].z =  cloud->points[cloud->size() - 1].z; 

  cloud_test->points[2].x =  c1.x; 
  cloud_test->points[2].y =  c1.y; 
  cloud_test->points[2].z =  c1.z; 
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = simpleVis(cloud_test, coefficients_cylinder);
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  } 
  return (0);
}

