#include <ros/ros.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/common/common_headers.h>

//header for filtering
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
//header for clustering purpose
#include <pcl/segmentation/conditional_euclidean_clustering.h>
//header for subscribing to kinect
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

//header for model coeffs-->optional
#include <pcl/ModelCoefficients.h>

//header for segmentation 
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/common/centroid.h>


#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
ros::Publisher pub, pub_plane;
typedef pcl::PointXYZ PointT;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
	
	sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*input, *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);	
		
	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;
  // for (size_t i = 0; i < 10; ++i)
  //       std::cerr << "    " << cloud->points[i] << std::endl;
	
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
  	pcl::VoxelGrid<pcl::PointXYZ> vg;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  	vg.setInputCloud (cloud);
  	vg.setLeafSize (0.01f, 0.01f, 0.01f);
  	vg.filter (*cloud_filtered);
  	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
  
  //stuff for model mathcing
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg1;
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::CentroidPoint<pcl::PointXYZ> centroid;// (new pcl::CentroidPoint<pcl::PointXYZ>);

	// Create the segmentation object for the planar model and set all the parameters
  	pcl::SACSegmentation<pcl::PointXYZ> seg;
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  	pcl::PCDWriter writer;
  	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (100);
  	seg.setDistanceThreshold (0.02);

  	int i=0, nr_points = (int) cloud_filtered->points.size ();
  	while (cloud_filtered->points.size () > 0.3 * nr_points)
  	{
    	// Segment the largest planar component from the remaining cloud
   	 	seg.setInputCloud (cloud_filtered);
    		seg.segment (*inliers, *coefficients);
    		if (inliers->indices.size () == 0)
   		 {
      			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      			break;
    		 }

    	// Extract the planar inliers from the input cloud
   	pcl::ExtractIndices<pcl::PointXYZ> extract;
    	extract.setInputCloud (cloud_filtered);
    	extract.setIndices (inliers);
    	extract.setNegative (false);

    	// Get the points associated with the planar surface
    	extract.filter (*cloud_plane);
    	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    	// Remove the planar inliers, extract the rest
    	extract.setNegative (true);
    	extract.filter (*cloud_f);
    	*cloud_filtered = *cloud_f;
  	}

  	// Creating the KdTree object for the search method of the extraction
 	 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud_filtered);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setClusterTolerance (0.02); // 2cm
  	ec.setMinClusterSize (100);
  	ec.setMaxClusterSize (2500);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud_filtered);
  	ec.extract (cluster_indices);
    std::vector<pcl::PointIndices>::const_iterator it;
	  std::vector<int>::const_iterator pit;
    std::cout << " ------------------------------------------------------------ " << std::endl;
    //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2 (new pcl::PointCloud<pcl::PointXYZ>);
    //std::cout << ec.getMinClusterSize () << std::endl;
    
    /*program to get the required point cloud out of all the clusters*/  
    int j = 0;
    int count = 0;
    int count_valid = 0;
    int valid = 0;
    std::cout << " ------------------------------------------------------------ " << std::endl;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
      std::cout << " ------------------------------------------------------------ " << std::endl;
      //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1 (new pcl::PointCloud<pcl::PointXYZ>);
      count_valid = 0;
      //if(count == 1){break;}
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        count_valid = 0;
        //if(count == 0){
        cloud_cluster1->points.push_back (cloud_filtered->points[*pit]);                          
        //}
        for (size_t i = 0; i < cloud_cluster1->points.size (); ++i){
          if((cloud_cluster1->points[i].x > -0.5) && (cloud_cluster1->points[i].x < 0.5))
          {
            count_valid++;
          }
        }              
      }
      
      if(count_valid > 120){
          
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
               cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
               }
      }
      //if(valid == 1){
      //  cloud_cluster->points.push_back (cloud_filtered->points); //
      //}
      // std::cout << "=========" << count_valid << "===========" << std::endl;
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      //if(count == 0){count++;}
      // std::cout << "AFTER PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      // std::cout << "AFTER PointCloud representing the Cluster1: " << cloud_cluster1->points.size () << " data points." << std::endl;
      // //std::cout << "Width Cluster: " << cloud_cluster->width << " data points." << std::endl;
      //std::cout << "Height Cluster: " << cloud_cluster->height << " data points." << std::endl;
      // for (size_t i = 0; i < 5; ++i)
      //    std::cout << "    " << cloud_cluster1->points[i] << std::endl;
      
      /*---getting cylinerical properties---*/

      // for (size_t i = 0; i < cloud_cluster1->points.size(); ++i)
      //     std::cout << cloud_cluster1->points[i].x << ' '<< cloud_cluster1->points[i].y <<' '<< cloud_cluster1->points[i].z << std::endl;
          
      cloud_cluster2->points = cloud_cluster1->points; 

      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;
      count++;
      //std::cout <<  " -- " <<(cloud_cluster1->points.size()) << std::endl;
    
    }
    std::cout << "Points in Cluster1: "<<(cloud_cluster2->points.size()) << std::endl;
    /*-----------------------------------------------------------------------------------------*/    
    /* cloud_cluster2 has the required point cloud, well get the cyllyinder paramters from it*/
      ne.setSearchMethod (tree1);
      ne.setInputCloud (cloud_cluster2);
      ne.setKSearch (50);
      ne.compute (*cloud_normals);

      seg1.setOptimizeCoefficients (true);
      seg1.setModelType (pcl::SACMODEL_CYLINDER);
      seg1.setMethodType (pcl::SAC_RANSAC);
      seg1.setNormalDistanceWeight (0.1);
      seg1.setMaxIterations (10000);
      seg1.setDistanceThreshold (0.05);
      seg1.setRadiusLimits (0, 0.1);
      seg1.setInputCloud (cloud_cluster2);
      seg1.setInputNormals (cloud_normals);

      seg1.segment (*inliers_cylinder, *coefficients_cylinder);
      std::cout << "Cylindrical Coefficients: " << std::endl;
      std::cout << "values[0:2]::Points on Cylindrical Axis " << std::endl;
      std::cout << "values[3:5]::Cylindrical Axis Direction " << std::endl;
      std::cout << "values[6]::RADIUS of Cylinder " << std::endl; 
      std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

      for (size_t i = 0; i < cloud_cluster2->points.size(); ++i){
        centroid.add(cloud_cluster2->points[i]);
      }

      pcl::PointXYZ c1;
      centroid.get (c1);
      std::cout << "Centroid of Cylinder[x,y,z]::||" <<c1.x << "||" << c1.y << "||" << c1.z << std::endl;
      
      /*feb-19*/
      /*getting transformation matrix from cylindrical axis*/
      /*conversion to quaternion*/








    /*-----------------------------------------------------------------------------------------*/
    /*----cloud_cluster2 is the extracted point cloud of cylinder----*/
    // pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_cy (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_cluster2->points));
    //pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_cy);

  // std::cout << type(cloud_cluster1->points) << std::endl;
  //cloud_filtered =  cloud_cluster1;
	pcl::toROSMsg (*cloud_cluster , *clusters);
  //pcl::toROSMsg (*cloud_filtered , *clusters);
	
	//clusters->header.frame_id = "/rgbd_camera_optical_frame";
	clusters->header.frame_id = "/camera_depth_frame";
	clusters->header.stamp=ros::Time::now();
	pub.publish (*clusters);

  /*extract the central cluster*/


}

int
main (int argc, char** argv)
{
  	// Initialize ROS
  	ros::init (argc, argv, "clust");
  	ros::NodeHandle nh;

  	// Create a ROS subscriber for the input point cloud
  	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  	// Create a ROS publisher for the output model coefficients
  	pub = nh.advertise<sensor_msgs::PointCloud2> ("clusters", 1);
		ros::spin ();
}