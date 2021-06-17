# mesh2cloud-ros
This is a ROS node that converts numpy voxel grid to a point cloud and transforms it to the robot frame.


#### Subscribed topics
The `mesh2cloud_node` node subscribes to the following topics:
*  `/camera/depth_registered/points (sensor_msgs::PointCloud2)` the robot's RGB-D point cloud.
*  `/predicted_pose_np (rospy_tutorials::Floats)` floats of the predicted pose (azimuth and elevation angles).
*  `/predicted_voxel_np (rospy_tutorials::Floats)` floats of the predicted shape (voxel grid).
*  `/bbox (rospy_tutorials::Floats)` floats of the bounding box containing the detected object.

#### Published topics
The `mesh2cloud_node` node publishes to the following topics:
*  `/predicted_mug_cloud (sensor_msgs::PointCloud2)` the input voxel grid converted into a point cloud and transformed to the robots frame.