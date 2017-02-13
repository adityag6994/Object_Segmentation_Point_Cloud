# tabletop_operation
$roscore 

$roslaunch openni_launch openni.launch (I have used a kinect xbox 360)

$rosrun tabletop_operation planar input:=/camera/depth/points (to view the planar segmentation)

$rosrun tabletop_operation cluster_euclid input:=/camera/depth/points  (to view the euclidean clustering)

$rosrun rviz rviz 

Change the default topic to camera_depth_frame. Also add a Pointcloud2 topic and subscribe to /output for planar segmentation and /clusters for euclidean clustering.


