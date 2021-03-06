// 作者：陈露
// current  version： V1.0 191219
// previous version： V1.0 191210
// 描述：point cloud odometry based on ICP

// ROS --------------------------------
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

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

#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <pcl/filters/filter.h>


#define PI 3.1415926


pcl::PointCloud<pcl::PointXYZ>::Ptr incloud(new pcl::PointCloud<pcl::PointXYZ>), source(new pcl::PointCloud<pcl::PointXYZ>), reference(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr incloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
bool isInitialized = false;

ros::Publisher referenceCloud_pub, readingCloud_pub;
ros::Publisher icpPoseVariation_pub, odomVariation_pub;


Eigen::Vector3d odomTCurrent;
Eigen::Vector4d odomRCurrent;



bool updateICP()
{
    // G-ICP
    // pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setTransformationEpsilon(0.01);
    // icp.setMaxCorrespondenceDistance(0.3);
    // icp.setMaximumIterations(30);
    // icp.setRANSACIterations(0);

    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    icp.setInputSource(source);
    icp.setInputTarget(reference);
    pcl::PointCloud<pcl::PointXYZ> unused;
    icp.align(unused); //这里kinect出来的有序点云貌似不可以直接进行demo里的半径过滤法会报错

    Eigen::Matrix4f transM = icp.getFinalTransformation();
    std::cout << "icp transformation = " << std::endl << icp.getFinalTransformation() << std::endl;


    // rotation matrix to quaternion
    Eigen::Matrix3d t_R;
    t_R(0,0) = transM(0,0);
    t_R(1,0) = transM(1,0);
    t_R(2,0) = transM(2,0);   
    t_R(0,1) = transM(0,1);
    t_R(1,1) = transM(1,1);
    t_R(2,1) = transM(2,1); 
    t_R(0,2) = transM(0,2);
    t_R(1,2) = transM(1,2);
    t_R(2,2) = transM(2,2);   
    std::cout << "rotation matrix = " << std::endl << t_R << std::endl;    

    Eigen::Quaterniond t_Q;
    t_Q = t_R;
    std::cout << "quaternion = " << std::endl << t_Q.coeffs() << std::endl;    


    // quaternion to euler_angle(ZYX-RPY)
    Eigen::Vector3d eulerAngle = t_Q.matrix().eulerAngles(2,1,0);
    std::cout << "euler angle in radian= " << std::endl << eulerAngle << std::endl;    
    Eigen::Vector3d eulerAngleInDegree = t_Q.matrix().eulerAngles(2,1,0);
    eulerAngleInDegree(0) = eulerAngle(0)*180/PI;
    eulerAngleInDegree(1) = eulerAngle(1)*180/PI;
    eulerAngleInDegree(2) = eulerAngle(2)*180/PI;        
    std::cout << "euler angle in degree= " << std::endl << eulerAngleInDegree << std::endl;     


    // translation
    Eigen::Vector3d translation;
    translation(0) = transM(0,3);
    translation(1) = transM(1,3);
    translation(2) = transM(2,3);
    std::cout << "translation = " << std::endl << translation << std::endl;   

    float positionDelta = 0.05, orientationDelta = PI/20;
    if(translation(0)>positionDelta || translation(1)>positionDelta || translation(2)>positionDelta)
    {
        ROS_ERROR("position changed large!");
    }

    if(eulerAngle(0)>orientationDelta || eulerAngle(1)>orientationDelta || eulerAngle(2)>orientationDelta)
    {
        ROS_WARN("orientation changed large!");
    }

    std_msgs::Float32MultiArray icpPoseVariation;
    icpPoseVariation.data.push_back(translation(0));  
    icpPoseVariation.data.push_back(translation(1));
    icpPoseVariation.data.push_back(translation(2));  
    icpPoseVariation.data.push_back(eulerAngleInDegree[0]);
    icpPoseVariation.data.push_back(eulerAngleInDegree[1]);
    icpPoseVariation.data.push_back(eulerAngleInDegree[2]);        
    icpPoseVariation_pub.publish(icpPoseVariation); 
}


void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    // pcl::PointCloud<pcl::PointXYZ>::ptr source(new pcl::PointCloud<pcl::PointXYZ>), reference(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *incloud); 
    
    if(!incloud->is_dense)
    {
        //ROS_WARN("input cloud is_dense=false!");
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*incloud, *incloudFiltered, indices);
    }
    else
    {
        *incloudFiltered = *incloud;
    }

    Eigen::Vector4d odomRK1, odomRK2; // quatertion
    Eigen::Vector3d odomTK1, odomTK2; // translation
    Eigen::Vector3d odomREK1, odomREK2; // euler angle
    if(!isInitialized)
    {
        pcl::copyPointCloud(*incloudFiltered, *source);

        odomTK1 = odomTCurrent;
        odomRK1 = odomRCurrent;

        isInitialized = true;
        return;
    }
    odomTK2 = odomTCurrent;
    odomRK2 = odomRCurrent;    
    std::cout << "odom position previous = " << odomTK1 << std::endl;
    std::cout << "odom orientation previous = " << odomRK1 << std::endl;
    std::cout << "odom position current = " << odomTK2 << std::endl;
    std::cout << "odom orientation current = " << odomRK2 << std::endl;
    std::cout << std::endl;    


    pcl::copyPointCloud(*source, *reference);
    pcl::copyPointCloud(*incloudFiltered, *source);

    // pub reference cloud & reading cloud.
    sensor_msgs::PointCloud2 referenceCloud, readingCloud;
    pcl::toROSMsg(*source, readingCloud);
    referenceCloud_pub.publish(readingCloud);
    pcl::toROSMsg(*reference, referenceCloud);
    readingCloud_pub.publish(referenceCloud);    

    updateICP();   

    // odom variation
    std_msgs::Float32MultiArray odomVariation;
    odomVariation.data.push_back(odomTK2[0] - odomTK1[0]);
    odomVariation.data.push_back(odomTK2[1] - odomTK1[1]);
    odomVariation.data.push_back(odomTK2[2] - odomTK1[2]);

    tf::Quaternion q( odomRK1[0], odomRK1[1], odomRK1[2], odomRK1[3]);
    tf::Matrix3x3 m(q);
    m.getRPY(odomREK1[0], odomREK1[1], odomREK1[2]);

    tf::Quaternion q2( odomRK2[0], odomRK2[1], odomRK2[2], odomRK2[3]);
    tf::Matrix3x3 m2(q2);
    m2.getRPY(odomREK2[0], odomREK2[1], odomREK2[2]);   
    odomVariation.data.push_back(odomREK2[0] - odomREK1[0]); 
    odomVariation.data.push_back(odomREK2[1] - odomREK1[1]);
    odomVariation.data.push_back(odomREK2[2] - odomREK1[2]);        

    // publish pose variation
    odomVariation_pub.publish(odomVariation);

    odomTK1 = odomTK2;
    odomRK1 = odomRK2;    
}


void odom_callback(const nav_msgs::OdometryConstPtr& odom)
{
    odomTCurrent(0) = odom->pose.pose.position.x;
    odomTCurrent(1) = odom->pose.pose.position.y;
    odomTCurrent(2) = odom->pose.pose.position.z;

    odomRCurrent(0) = odom->pose.pose.orientation.x;
    odomRCurrent(1) = odom->pose.pose.orientation.y;
    odomRCurrent(2) = odom->pose.pose.orientation.z;
    odomRCurrent(3) = odom->pose.pose.orientation.w;    
}


// ========================================= publisher

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "icp_pose");
    ros::NodeHandle nh("~");
    //ros::NodeHandle nh;

    // Subscriber
    //ros::Subscriber cloud_sub = nh.subscribe("/bluerov2h_follower/depth/points2", 10, &pointcloud_callback);  
    ros::Subscriber cloud_sub = nh.subscribe("/cloud_centroid/radius_outlier_removal_points2", 10, &pointcloud_callback);   
    ros::Subscriber odom_sub = nh. subscribe("/bluerov2h_leader/pose_gt", 20, &odom_callback);


    // publisher
    referenceCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("reference_cloud", 10);
    readingCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("reading_cloud", 10);
    icpPoseVariation_pub = nh.advertise<std_msgs::Float32MultiArray>("icp_pose_variation", 10);
    odomVariation_pub = nh.advertise<std_msgs::Float32MultiArray>("odom_variation", 10);


    ros::AsyncSpinner spinner(2);  
    ros::Rate rate(1);
    spinner.start();
    while(ros::ok())
    {
        ROS_INFO("*********************************************");        
        //spinner.start();
        ros::spinOnce();
        rate.sleep();
    }  

    spinner.stop();

    return 0;
}
