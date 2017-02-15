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
  viewer->setBackgroundColor (0, 0, 0);
  //void pcl::visualization::PCLVisualizer::setBackgroundColor  (   const double &    r, 
  //bool pcl::visualization::PCLVisualizer::addSphere   (   const PointT &    center, 
  // viewer->addSphere<pcl::PointXYZ> ()
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.3);
  viewer->initCameraParameters ();
  viewer->addLine<pcl::PointXYZ> (cloud->points[0], cloud->points[1], "line");
  
  viewer->addSphere (cloud->points[0], 0.01, "sphere");

  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (coefficients_cylinder->values[0]);
  coeffs.values.push_back (coefficients_cylinder->values[1]);
  coeffs.values.push_back (-0.782631);//coefficients_cylinder->values[2]);
  coeffs.values.push_back (coefficients_cylinder->values[3]);
  coeffs.values.push_back (coefficients_cylinder->values[4]);
  coeffs.values.push_back (coefficients_cylinder->values[5]);
  coeffs.values.push_back (coefficients_cylinder->values[6]);
  viewer->addCylinder (coeffs, "cylinder");

  //return (viewer);
  
  return (viewer);
}

// boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::ModelCoefficients::ConstPtr coefficients_cylinder)
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//   viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();

//   //------------------------------------
//   //-----Add shapes at cloud points-----
//   //------------------------------------
//   viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
//                                      cloud->points[cloud->size() - 1], "line");
//   viewer->addSphere (cloud->points[0], 0.2, "sphere");

//   //---------------------------------------
//   //-----Add shapes at other locations-----
//   //---------------------------------------
//   pcl::ModelCoefficients coeffs;
//   // coeffs.values.push_back (0.0);
//   // coeffs.values.push_back (0.0);
//   // coeffs.values.push_back (1.0);
//   // coeffs.values.push_back (0.0);
//   // viewer->addPlane (coeffs, "plane");
//   // coeffs.values.clear ();
//   coeffs.values.push_back (coefficients_cylinder->values[0]);
//   coeffs.values.push_back (coefficients_cylinder->values[1]);
//   coeffs.values.push_back (coefficients_cylinder->values[2]);
//   coeffs.values.push_back (coefficients_cylinder->values[3]);
//   coeffs.values.push_back (coefficients_cylinder->values[4]);
//   coeffs.values.push_back (coefficients_cylinder->values[5]);
//   coeffs.values.push_back (coefficients_cylinder->values[6]);
//   viewer->addCylinder (coeffs, "cylinder");

//   return (viewer);
// }

// boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
//   viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//   // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();

//   //------------------------------------
//   //-----Add shapes at cloud points-----
//   //------------------------------------
//   viewer->addLine<pcl::PointXYZ> (cloud->points[0],
//                                      cloud->points[cloud->size() - 1], "line");
//   viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

//   //---------------------------------------
//   //-----Add shapes at other locations-----
//   //---------------------------------------
//   pcl::ModelCoefficients coeffs;
//   coeffs.values.push_back (0.0);
//   coeffs.values.push_back (0.0);
//   coeffs.values.push_back (1.0);
//   coeffs.values.push_back (0.0);
//   viewer->addPlane (coeffs, "plane");
//   coeffs.values.clear ();
//   coeffs.values.push_back (0.3);
//   coeffs.values.push_back (0.3);
//   coeffs.values.push_back (0.0);
//   coeffs.values.push_back (0.0);
//   coeffs.values.push_back (1.0);
//   coeffs.values.push_back (0.0);
//   coeffs.values.push_back (5.0);
//   viewer->addCone (coeffs, "cone");

//   return (viewer);
// }


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
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0, 1.5);
  // pass.filter (*cloud_filtered);
  // std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  // seg.setNormalDistanceWeight (0.1);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.03);
  // seg.setInputCloud (cloud_filtered);
  // seg.setInputNormals (cloud_normals);
  // // Obtain the plane inliers and coefficients
  // seg.segment (*inliers_plane, *coefficients_plane);
  // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // // Extract the planar inliers from the input cloud
  // extract.setInputCloud (cloud_filtered);
  // extract.setIndices (inliers_plane);
  // extract.setNegative (false);

  // // Write the planar inliers to disk
  // pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  // extract.filter (*cloud_plane);
  // std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  // writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // // Remove the planar inliers, extract the rest
  // extract.setNegative (true);
  // extract.filter (*cloud_filtered2);
  // extract_normals.setNegative (true);
  // extract_normals.setInputCloud (cloud_normals);
  // extract_normals.setIndices (inliers_plane);
  // extract_normals.filter (*cloud_normals2);

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

  
  std::cout << cloud->points[0] << endl;
  // std::cout << type(cloud->points) << endl;
  pcl::PointCloud<PointT>::Ptr cloud_test (new pcl::PointCloud<PointT> ());
  *cloud_test = *cloud;
  cloud_test->points[0].x =  coefficients_cylinder->values[0]; 
  cloud_test->points[0].y =  coefficients_cylinder->values[1]; 
  cloud_test->points[0].z = -coefficients_cylinder->values[2]; 
  cloud_test->points[1].x =  cloud->points[cloud->size() - 1].x; 
  cloud_test->points[1].y =  cloud->points[cloud->size() - 1].y; 
  cloud_test->points[1].z =  cloud->points[cloud->size() - 1].z; 
  
  std::cout << cloud_test->points[0] << endl;
  std::cout << cloud_test->points[1] << endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = simpleVis(cloud_test, coefficients_cylinder);
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  } 
  return (0);
}

  // values[0]:   0.0539518
  // values[1]:   0.0928493
  // values[2]:   0.782631
  // values[3]:   0.0247147
  // values[4]:   -0.83652
  // values[5]:   -0.547379
  // values[6]:   0.038769
