
#include <iostream>

#include <ros/ros.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
//header for filtering
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
//header for clustering purpose
#include <pcl/segmentation/conditional_euclidean_clustering.h>
//header for subscribing to kinect
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//header for model coeffs-->optional
#include <pcl/ModelCoefficients.h>
//header for segmentation 
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


using namespace std;

ros::Publisher pub, pub_plane;

/*Genral Information

	38304 data points
	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;
	
	cout  << cloud->sensor_orientation_.matrix () << endl;
	1 0 0
	0 1 0
	0 0 1

	
*/

int _flag=0; //keep track of progeress

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
  _flag++;

  // cout << endl;
  // if(_flag%50 == 0)
  // cout << "_________________________________________________________________" << endl;
/*cloud: input point cloud in PointXYZ*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*input, *cloud);
	// if(_flag%50 == 0)
  // std::cout << "START: " << cloud->points.size () << " data points." << std::endl;

/*********Downsampling******************/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;// (new pcl::VoxelGrid<pcl::PointXYZ>);
 	sor.setInputCloud (cloud);
 	sor.setLeafSize (0.005f, 0.005f, 0.005f);
 	sor.filter (*cloud_filtered);
  // if(_flag%50 == 0)
 	// std::cout << "DOWNSAMLING: " << cloud_filtered->points.size () << " data points." << std::endl;

/*********plane extraction**U***********/
 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
 	pcl::SACSegmentation<pcl::PointXYZ> seg;
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  	//pcl::PCDWriter writer;
  	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (100);
  	seg.setDistanceThreshold (0.09);
  	int i=0, nr_points = (int) cloud_filtered->points.size ();
  	
  	while (cloud_filtered->points.size () > 0.3 * nr_points){

    	// Segment the largest planar component from the remaining cloud
   	 	seg.setInputCloud (cloud_filtered);
    		seg.segment (*inliers, *coefficients);
    		if (inliers->indices.size () == 0){
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
    	// std::cout << "PLANARCOMPONENT: " << cloud_plane->points.size () << " data points." << std::endl;

    	// Remove the planar inliers, extract the rest
    	extract.setNegative (true);
    	extract.filter (*cloud_f);
    	*cloud_filtered = *cloud_f;
  	}

    // if(_flag%50 == 0){
    //       std::cout << "AFTER PLANARCOMPONENT: " << cloud_filtered->points.size () << " data points." << std::endl;
    // }
    // for(size_t i=0 ; i < 10 ; i++){
    // cout << cloud_filtered->points[i].x << " . " << cloud_filtered->points[i].y << " . " << cloud_filtered->points[i].z  << endl;
    // }

/*********Eucledean Clustering*U********/
// cout << "__________________________" << endl;
// cout << endl;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_cluster (new pcl::PointCloud<pcl::PointXYZ>); 
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
tree->setInputCloud (cloud_filtered);
std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec1;
ec1.setClusterTolerance (0.02); // 2cm
ec1.setMinClusterSize (300);
ec1.setMaxClusterSize (2500);
ec1.setSearchMethod (tree);
ec1.setInputCloud (cloud_filtered);
ec1.extract (cluster_indices);
std::vector<pcl::PointIndices>::const_iterator it;
std::vector<int>::const_iterator pit;

int j = 0; 
for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    j++;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {

          cloud_cluster->points.push_back(cloud_filtered->points[*pit]); 
      
    }

    // if(_flag%50 == 0) 
    // std::cout << "CLUSTER  " << j << " : " << cloud_cluster->points.size () << " data points." << std::endl;
    //*cloud_filtered_cluster += *cloud_cluster;
    *cloud_filtered += * cloud_cluster;
}

// /***********model matching*************/
//   pcl::PassThrough<pcl::PointXYZ> pass;
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg1;
//   pcl::ExtractIndices<pcl::PointXYZ> extract_1;
//   pcl::ExtractIndices<pcl::Normal> extract_normals;
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
//   pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
//   pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

//   //normal calculations
//   ne.setSearchMethod (tree);
//   ne.setInputCloud (cloud_filtered);
//   ne.setKSearch (50);
//   ne.compute (*cloud_normals);

//   //ransac mddel fitting
//   seg1.setOptimizeCoefficients (true);
//   seg1.setModelType (pcl::SACMODEL_CYLINDER);
//   seg1.setMethodType (pcl::SAC_RANSAC);
//   seg1.setNormalDistanceWeight (0.1);
//   seg1.setMaxIterations (100);
//   seg1.setDistanceThreshold (0.05);
//   seg1.setRadiusLimits (0, 0.1);
//   seg1.setInputCloud (cloud_filtered);
//   seg1.setInputNormals (cloud_normals);
//   seg1.segment (*inliers_cylinder, *coefficients_cylinder);
//   std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

/***********Centroid*********************/
// pcl::CentroidPoint<pcl::PointXYZ> centroid;
// for (size_t i = 0; i < cloud_filtered->points.size(); ++i){
//      centroid.add(cloud->points[i]);
// }
// pcl::PointXYZ c1;
// centroid.get (c1);
// if(_flag%50 == 0)
// std::cout << "||" <<c1.x << "||" << c1.y << "||" << c1.z << std::endl;
float centroid[3]={};
for(size_t i=0; i< cloud_filtered->points.size() ; i++){
    centroid[0] += cloud_filtered->points[i].x;
    centroid[1] += cloud_filtered->points[i].y;
    centroid[2] += cloud_filtered->points[i].z;
}

centroid[0] = centroid[0]  / cloud_filtered->points.size();
centroid[1] = centroid[1]  / cloud_filtered->points.size();
centroid[2] = centroid[2]  / cloud_filtered->points.size();

// if(_flag%50 == 0){
  // cout << "size :: " << cloud_filtered->points.size() << endl;
  cout << centroid[0] <<',' << centroid[1] << ',' << centroid[2]<< endl;
//   cout << "y: " << centroid[1] << endl;
//   cout << "z: " << centroid[2] << endl;
// // }
// /*************publishing the output**/
//   cout << "_________________________________________________________________" << endl;
//   std::cout << "AFTER MODELMATCHING: " << cloud_filtered->points.size () << " data points." << std::endl;
//   cout << endl;
// if(_flag%50 == 0){
//   for(size_t i=0 ; i < 200 ; i++){
//     cout << cloud_filtered->points[i].x << " . " << cloud_filtered->points[i].y << " . " << cloud_filtered->points[i].z  << endl;
//   }
// }




// if(_flag%50 == 0)
// cout << endl;

/*output the point cloud as clusters*/	
		
	pcl::toROSMsg (*cloud_filtered, *clusters);
	clusters->header.frame_id = "/pico_flexx_optical_frame";
	clusters->header.stamp=ros::Time::now();
	pub.publish (*clusters);
}


int
main (int argc, char** argv)
{
  
  ros::init (argc, argv, "pico_flexx_sub");
  ros::NodeHandle nh;
	
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/pico_flexx/points", 1, cloud_cb);

  //_flag++;

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  ros::spin ();
}
