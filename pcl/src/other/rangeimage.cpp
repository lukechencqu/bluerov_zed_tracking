// 作者：陈露
// current  version： V1.0 191121
// previous version： V1.0 191119-2
// 描述：cloud cluster, bounding box, center and centroid

// ROS --------------------------------
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>

// C++ ------------------------------
#include <iostream>
#include <boost/thread/thread.hpp>

// eigen -----------------------------
#include <Eigen/Dense>

// pcl -------------------------------
#include <pcl/common/common_headers.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>


#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>




void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	//设置sensor_pose和coordinate_frame
	Eigen::Affine3f sensorPose;//设置相机位姿
	sensorPose.setIdentity(); //成像时遵循的坐标系统
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel=0.00;//设置噪声水平
	float minRange = 0.0f;//成像时考虑该阈值外的点
	int width=640,height=480,size=2,type=0;
	float fx=525,fy=525,cx=320,cy=240;
 
	pcl::RangeImagePlanar::Ptr rangeImage(new pcl::RangeImagePlanar);
	rangeImage->createFromPointCloudWithFixedSize(*cloud,width,height,cx,cy,fx,fy,sensorPose,coordinate_frame);
	std::cout << rangeImage << "\n";

	//深度图像可视化
	pcl::visualization::RangeImageVisualizer range_image_widget ("rangeImage");
	range_image_widget.showRangeImage (*rangeImage);
	range_image_widget.setWindowTitle("PCL_rangeImage");    

}


// ========================================= publisher

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "cloud_centroid");
    ros::NodeHandle nh("~");
    //ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber cloud_sub = nh.subscribe("/bluerov2h_follower/depth/points2", 10, &pointcloud_callback);   


    ros::AsyncSpinner spinner(2);  
    ros::Rate rate(1);
    spinner.start();
    while(ros::ok())
    {
        ROS_INFO_THROTTLE(1, "main while...");        
        //spinner.start();
        ros::spinOnce();
        rate.sleep();
    }  

    spinner.stop();

    return 0;
}
