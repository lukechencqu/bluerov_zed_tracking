// author chenlu
// version 2020-0117-1

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/common/intersections.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
// filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
// features
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/search/kdtree.h>
// segment
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

#include <iostream>
#include <Eigen/Dense>

#define PI 3.1415926
//#define DEBUG
boost::mutex cloudMutex;

// ========================================= publisher
ros::Publisher passthroughfilter_points_pub;
ros::Publisher downsample_points_pub;
ros::Publisher radiusOutlierRemovalPoints_pub, radiusOutlierRemovalPoints2_pub;
ros::Publisher boundingBox_pub, boundingBoxCentroidMarker_pub, boundingBoxCenterMarker_pub;
ros::Publisher cloud_pub, odom_pub;
ros::Publisher referenceCloud_pub, readingCloud_pub, alignedCloud_pub;
ros::Publisher fusedVariation_pub, bbxVariation_pub, icpPoseVariation_pub, odomVariation_pub, poseVariationDiff_pub, pk2p0_pub;


// ========================================= struct
struct Dectected_Obj
{
  jsk_recognition_msgs::BoundingBox boundingBox_;

  pcl::PointXYZ minPoint_;
  pcl::PointXYZ maxPoint_;
  pcl::PointXYZ centroid_;
  //pcl::PointXYZ center_;
  Eigen::Vector3f center_;
};

// ========================================= global variables
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), reference(new pcl::PointCloud<pcl::PointXYZ>), source(new pcl::PointCloud<pcl::PointXYZ>);  
pcl::PointCloud<pcl::PointXYZ>::Ptr passthoughfilter_points(new pcl::PointCloud<pcl::PointXYZ>);   
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_points(new pcl::PointCloud<pcl::PointXYZ>);    
pcl::PointCloud<pcl::PointXYZ>::Ptr radiusOutlierRemovalPoints (new pcl::PointCloud<pcl::PointXYZ>), radiusOutlierRemovalPoints2 (new pcl::PointCloud<pcl::PointXYZ>);   
pcl::PointCloud<pcl::PointXYZ>::Ptr incloud(new pcl::PointCloud<pcl::PointXYZ>), incloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>), reference(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr alignedCloud(new pcl::PointCloud<pcl::PointXYZ>);

bool newCloudFlag = false;


static geometry_msgs::Point odomInitialPose, odomCurrentCalledPose, odomPreviousCalledPose;
static geometry_msgs::Point pointcloud_initial_pose, cloud_current_pose, cloud_previous_pose;
geometry_msgs::Pose odomPoseChanged, pointcloudPoseChanged, poseChangedBias, poseChangedBias_current, poseChangedBias_previous;
geometry_msgs::Pose odomPoseDelta, odomPoseCurrent, odomPosePrevious, odomPoseAccumulate;
std::string odomPoseTopic, cloudTopic;
bool cloudFilterSucceed = false;
int pointcloudPoseChangedMsgID = 0;
bool isInitialized = false;

Eigen::Vector3d odomTCurrent;
Eigen::Vector4d odomRCurrent;
Eigen::Vector4d odomRK1, odomRK2; // quatertion
Eigen::Vector3d odomTK1, odomTK2; // translation
Eigen::Vector3d odomREK1, odomREK2; // euler angle
Eigen::Vector3f bbxTK1, bbxTK2;

float threshold_X=0.03, threshold_Y=0.03, threshold_Z = 0.03;
int frameNumSet = 10;
int frameNum = 0;
Eigen::Vector3f pSum, pMean; // x,y,z
bool initialized = false;
Eigen::Vector3f initialPose;


float odomTime, odomX, odomY, odomZ;
bool odomStart = true, cloudDone = false;
ros::Time cloudSubTime;

// ========================================= parameters
double field_limit_x1, field_limit_x2, field_limit_y1, field_limit_y2, field_limit_z1, field_limit_z2;
// downsample
double leaf_size;
// SACSegmentation
int model_type;
int method_type;
int horizontalplane_method_type, horizontalplane_model_type, verticalplane_method_type, verticalplane_model_type;
bool setOptimizeCoefficients;
double horizontalplane_eps_angle_in_degreer, horizontalplane_setDistanceThreshold, verticalplane_eps_angle_in_degreer, verticalplane_setDistanceThreshold;
double setRadiusSearch, setMinNeighborsInRadius, setRadiusSearch2, setMinNeighborsInRadius2;
// cluster
double clusterTolerance, clusterMinSize, clusterMaxSize;



//**************************************************** MODULE function ************************************************************

bool read_parameters()
{
  // 从参数服务器获取参数
  // passthrough filter parameters
  if(!ros::param::get("~passthrough_filter/field_limit_x1", field_limit_x1))
    ROS_WARN_STREAM("could not get field_limit_x1 from parameterserver!");    
  
  if(!ros::param::get("~passthrough_filter/field_limit_x2", field_limit_x2))
    ROS_WARN_STREAM("could not get field_limit_x2 from parameterserver!");  
  
  if(!ros::param::get("~passthrough_filter/field_limit_y1", field_limit_y1))
    ROS_WARN_STREAM("could not get field_limit_y1 from parameterserver!");    
  
  if(!ros::param::get("~passthrough_filter/field_limit_y2", field_limit_y2))
    ROS_WARN_STREAM("could not get field_limit_y2 from parameterserver!");   
  
  if(!ros::param::get("~passthrough_filter/field_limit_z1", field_limit_z1))
    ROS_WARN_STREAM("could not get field_limit_z1 from parameterserver!");    
  
  if(!ros::param::get("~passthrough_filter/field_limit_z2", field_limit_z2))
    ROS_WARN_STREAM("could not get field_limit_z2 from parameterserver!");   
    
  // downsample filter parameters
  if(!ros::param::get("~downsample/leaf_size", leaf_size))
    ROS_WARN_STREAM("could not get leaf_size from parameterserver!");    
    
  // SACSegmentation  
  if(!ros::param::get("~SACSegmentation/setOptimizeCoefficients", setOptimizeCoefficients))
    ROS_WARN_STREAM("could not get setOptimizeCoefficients from parameterserver!");      
  
  // RadiusOutlierRemoval
  if(!ros::param::get("~RadiusOutlierRemoval/setRadiusSearch", setRadiusSearch))
    ROS_WARN_STREAM("could not get setRadiusSearch from parameterserver!"); 
  
  if(!ros::param::get("~RadiusOutlierRemoval/setMinNeighborsInRadius", setMinNeighborsInRadius))
    ROS_WARN_STREAM("could not get setMinNeighborsInRadius from parameterserver!");     
  
  if(!ros::param::get("~RadiusOutlierRemoval/setRadiusSearch2", setRadiusSearch2))
    ROS_WARN_STREAM("could not get setRadiusSearch2 from parameterserver!"); 
  
  if(!ros::param::get("~RadiusOutlierRemoval/setMinNeighborsInRadius2", setMinNeighborsInRadius2))
    ROS_WARN_STREAM("could not get setMinNeighborsInRadius2 from parameterserver!");   

  // topic name
  if(!ros::param::get("~Topics/cloudTopic", cloudTopic))
    ROS_WARN_STREAM("could not get cloudTopic from parameterserver!");   
  //ROS_INFO_STREAM("cloudTopic: " << cloudTopic);

  // cluster
  if(!ros::param::get("~cluster/clusterTolerance", clusterTolerance))
    ROS_WARN_STREAM("could not get clusterTolerance from parameterserver!"); 
  if(!ros::param::get("~cluster/clusterMinSize", clusterMinSize))
    ROS_WARN_STREAM("could not get clusterMinSize from parameterserver!");   
  if(!ros::param::get("~cluster/clusterMaxSize", clusterMaxSize))
    ROS_WARN_STREAM("could not get clusterMaxSize from parameterserver!");         
   
    
  //ROS_WARN_STREAM("normal_kdtree_searchradius: " << normal_kdtree_searchradius << "  normal_kdtree_searchnum: " << normal_kdtree_searchnum);    
  
  return true;
}


void passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, 
    double field_limit_x1, double field_limit_x2, double field_limit_y1, double field_limit_y2, double field_limit_z1, double field_limit_z2, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr &passthrough_filter_out)
{
  pcl::PassThrough<pcl::PointXYZ> pass_filter;  
  
  // X方向滤波
  // 首先，在机器人前进方向（x方向）进行滤波，设置一个ROI区域,距离机器人太远的和太近的点云都将被滤除掉
  if(field_limit_x1 != 0.0 && field_limit_x2 != 0.0)
  {    
    pass_filter.setInputCloud(points);
    
    pass_filter.setFilterFieldName("x");
    pass_filter.setFilterLimits(field_limit_x1, field_limit_x2); // ！根据真实场景调整参数

    pass_filter.filter(*passthrough_filter_out);     
  }

  // Y方向滤波  
  if(field_limit_y1 != 0.0 && field_limit_y2 != 0.0)
  {    
    if(field_limit_x1 != 0.0 && field_limit_x2 != 0.0)  
      pass_filter.setInputCloud(passthrough_filter_out);
    else
      pass_filter.setInputCloud(points);
    
    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(field_limit_y1, field_limit_y2); // ！根据真实场景调整参数

    pass_filter.filter(*passthrough_filter_out);      
  }
  
  // Z方向滤波  
  // 然后，在高度方向（z方向）进行滤波，将地面点云滤除掉  
  if(field_limit_z1 != 0.0 && field_limit_z2 != 0.0)
  {    
    if( (field_limit_x1 != 0.0 && field_limit_x2 != 0.0) || (field_limit_y1 != 0.0 && field_limit_y2 != 0.0) )  
      pass_filter.setInputCloud(passthrough_filter_out);
    else
      pass_filter.setInputCloud(points);
    
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(field_limit_z1, field_limit_z2); // ！根据真实场景调整参数

    pass_filter.filter(*passthrough_filter_out);
  }
  
}


void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, 
    double leaf_size, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr &downsample_out)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxcelgrid_filter;
  
  voxcelgrid_filter.setInputCloud(points);
  
  voxcelgrid_filter.setLeafSize(leaf_size, leaf_size, leaf_size); // ！根据真实场景调整参数
  
  voxcelgrid_filter.filter(*downsample_out);    
}


std::vector<Dectected_Obj> clusterSegmentBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    double clusterTolerance, double clusterMinSize, double clusterMaxSize)
{
    // std::cout<<"\ncluster segment Bounding Box......"<<std::endl;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ros::Time startTime = ros::Time::now();
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(clusterMinSize);
    ec.setMaxClusterSize(clusterMaxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
    ROS_INFO_STREAM("cluster numbers: " << clusterIndices.size());
    // ros::Time endTime1 = ros::Time::now();
    // ROS_INFO_STREAM("time1 = " << endTime1 - startTime);    

    std::vector<Dectected_Obj> objList;  
    for(size_t i=0; i<clusterIndices.size(); i++)
    {
        Dectected_Obj detectedObj;

        float minX = std::numeric_limits<float>::max();
        float maxX = -std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        float maxY = -std::numeric_limits<float>::max();
        float minZ = std::numeric_limits<float>::max();
        float maxZ = -std::numeric_limits<float>::max();        

        // for(std::vector<int>::const_iterator pit = it)
        for(auto pit = clusterIndices[i].indices.begin(); pit != clusterIndices[i].indices.end(); ++pit)
        {
          pcl::PointXYZ p;
          p.x = cloud->points[*pit].x;
          p.y = cloud->points[*pit].y;
          p.z = cloud->points[*pit].z;

          detectedObj.centroid_.x += p.x;
          detectedObj.centroid_.y += p.y;
          detectedObj.centroid_.z += p.z;

          if(p.x < minX) minX = p.x; 
          if(p.y < minY) minY = p.y; 
          if(p.z < minZ) minZ = p.z; 
          if(p.x > maxX) maxX = p.x; 
          if(p.y > maxY) maxY = p.y; 
          if(p.z > maxZ) maxZ = p.z;       
        }

        // fill detected object information
        // detectedObj.boundingBox_.header = cloud->header;
        detectedObj.boundingBox_.header.seq = cloud->header.seq;
        // detectedObj.boundingBox_.header.stamp = cloud->header.stamp;
        detectedObj.boundingBox_.header.frame_id = cloud->header.frame_id;

        detectedObj.minPoint_.x = minX;
        detectedObj.minPoint_.y = minY;
        detectedObj.minPoint_.z = minZ;
        detectedObj.maxPoint_.x = maxX;
        detectedObj.maxPoint_.y = maxY;
        detectedObj.maxPoint_.z = maxZ;    

        // mass centroid
        if(clusterIndices[i].indices.size() > 0)
        {
          detectedObj.centroid_.x /= clusterIndices[i].indices.size();
          detectedObj.centroid_.y /= clusterIndices[i].indices.size();
          detectedObj.centroid_.z /= clusterIndices[i].indices.size();
        }

        // shape center
        Eigen::Vector4f minP1, maxP1;
        pcl::PointXYZ minP1_1, maxP1_1;
        pcl::getMinMax3D(*cloud, clusterIndices[i], minP1, maxP1);
        minP1_1.x = minP1[0];
        minP1_1.y = minP1[1];
        minP1_1.z = minP1[2];
        maxP1_1.x = maxP1[0];
        maxP1_1.y = maxP1[1];
        maxP1_1.z = maxP1[2];     
        detectedObj.center_ = 0.5f * ( minP1_1.getVector3fMap() + maxP1_1.getVector3fMap() );

        // bounding box
        double length = detectedObj.maxPoint_.x - detectedObj.minPoint_.x;
        double width = detectedObj.maxPoint_.y - detectedObj.minPoint_.y;
        double height = detectedObj.maxPoint_.z - detectedObj.minPoint_.z;

        detectedObj.boundingBox_.pose.position.x = detectedObj.minPoint_.x + length/2;
        detectedObj.boundingBox_.pose.position.y = detectedObj.minPoint_.y + width/2;
        detectedObj.boundingBox_.pose.position.z = detectedObj.minPoint_.z + height/2;

        detectedObj.boundingBox_.dimensions.x = ((length<0) ? -1*length : length);
        detectedObj.boundingBox_.dimensions.y = ((width<0) ? -1*width : width);
        detectedObj.boundingBox_.dimensions.z = ((height<0) ? -1*height : height);

        objList.push_back(detectedObj);
        // ros::Time endTime2 = ros::Time::now();
        // ROS_INFO_STREAM("time2 = " << endTime2 - startTime);          
    }
    // std::cout<<"cluster segment Bounding Box end.\n"<<std::endl;

    return(objList);
}


Eigen::Matrix4f updateICP()
{
    // G-ICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setTransformationEpsilon(0.01);
    icp.setMaxCorrespondenceDistance(0.3);
    icp.setMaximumIterations(100);
    icp.setRANSACIterations(0);

    // ICP
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    icp.setInputSource(source);
    icp.setInputTarget(reference);
    //pcl::PointCloud<pcl::PointXYZ> alignedCloud;
    icp.align(*alignedCloud); 

    if(icp.hasConverged())
    {
      std::cout << "\nICP has converged, score= " << icp.getFitnessScore() << std::endl;
    }

    Eigen::Matrix4f transM = icp.getFinalTransformation();
    //std::cout << "icp transformation = " << std::endl << icp.getFinalTransformation() << std::endl;
    return transM;
}

void odomCallback(const nav_msgs::OdometryConstPtr& odomIn)
{
  std::cout<<"odom callback"<<std::endl;
  if(odomStart==true || cloudDone==true){

    odomTime = odomIn->header.stamp.toSec();
    odomX = odomIn->pose.pose.position.x;
    odomY = odomIn->pose.pose.position.y;
    odomZ = odomIn->pose.pose.position.z;
  }
  odomStart = false;
  cloudDone = false;

/*   odomCloud.data.push_back(time);
  odomCloud.data.push_back(odomIn->pose.pose.position.x);
  odomCloud.data.push_back(odomIn->pose.pose.position.y);
  odomCloud.data.push_back(odomIn->pose.pose.position.z); */
}

/*
  get a stablized object's initial pose in point_cloud frame.
*/
bool initialization(std_msgs::Float32MultiArray delta_P, Eigen::Vector3f p_k)
{
  
    
    float delta_X = 0.0, delta_Y = 0.0, delta_Z=0.0;
    delta_X = delta_P.data[0];
    delta_Y = delta_P.data[1];
    delta_Z = delta_P.data[2];

    bool if_X = 0, if_Y = 0, if_Z = 0;
    if(delta_X<threshold_X)
      if_X = 1;
    if(delta_Y<threshold_Y)
      if_Y = 1;
    if(delta_Z<threshold_Z)
      if_Z = 1;

    if(if_X==1 && if_Y==1 && if_Z==1)
    {
      ++frameNum;
      pSum[0] = pSum[0] + p_k[0];
      pSum[1] = pSum[1] + p_k[1];
      pSum[2] = pSum[2] + p_k[2];
    }
    else
    {
      frameNum = 0;
      pSum[0] = 0.0;
      pSum[1] = 0.0;
      pSum[2] = 0.0;
    }
    
    if(frameNum > frameNumSet)
    { 
      pMean[0] = pSum[0]/frameNum;
      pMean[1] = pSum[1]/frameNum;
      pMean[2] = pSum[2]/frameNum;

      return 1;
    }

    return 0;
}


void callback(const sensor_msgs::PointCloud2ConstPtr& cloudIn)
{
    std::cout << "cloud point CALL_BACK()" << std::endl;

    if(0 == cloud->points.size())
    {
      ROS_ERROR("ERROR! input cloud size is 0 !");
      cloudFilterSucceed = false;
      //return;
    }
    cloudFilterSucceed = true;

    cloudMutex.lock();
    cloudSubTime = cloudIn->header.stamp;
    pcl::fromROSMsg(*cloudIn, *cloud);     // sensor_msgs::PointCloud2类型转换为pcl::PointCloud<pcl::PointXYZ>类型  
    ROS_INFO_STREAM("initial cloud information    points size: " << cloud->points.size() << "  frame id: " << cloud->header.frame_id);         
    cloudMutex.unlock();    

    newCloudFlag = true;
}


void cloudFun()
{
    std::cout << "\n*************cloud process begin***********************" << std::endl;

    //std::cout.precision(3);
    std::cout<<std::fixed<<std::setprecision(3)<<std::endl;    

    ros::Time beginTime = ros::Time::now();		
    // ============================================== pass filter and downsample
    if((field_limit_x1 != 0.0 && field_limit_x2 != 0.0) || (field_limit_y1 != 0.0 && field_limit_y2 != 0.0) || (field_limit_z1 != 0.0 && field_limit_z2 != 0.0))  // passfilter before downsample
    {  
        //delta_x_filter = 0.0; 
        std::cout<<"passthrough filter param: x1="<<field_limit_x1<<" x2="<<field_limit_x2
          <<" y1="<<field_limit_y1<<" y2="<<field_limit_y2<<" z1="<<field_limit_z1<<" z2="<<field_limit_z2<<std::endl;
        
        // pass filter
        passthrough_filter(cloud, field_limit_x1, field_limit_x2, field_limit_y1, field_limit_y2, field_limit_z1, field_limit_z2, passthoughfilter_points);
        if(0 == passthoughfilter_points->points.size()) // 直接跳出回调函数
        {
          ROS_ERROR_STREAM("ERROR! passthoughfilter points size is 0!");
          cloudFilterSucceed = false;
          //return;
        }         
        
        sensor_msgs::PointCloud2 passthroughfilter_points_rosmsg;
        pcl::toROSMsg(*passthoughfilter_points, passthroughfilter_points_rosmsg);
        passthroughfilter_points_pub.publish(passthroughfilter_points_rosmsg);    
        
        // voxelgrid downsample
        downsample(passthoughfilter_points, leaf_size, downsample_points);    
    }
    else  // downsample without passfilter
    {
        downsample(cloud, leaf_size, downsample_points);    
    }
    // publish downsample pointcloud
    sensor_msgs::PointCloud2 downsample_points_rosmsg;
    pcl::toROSMsg(*downsample_points, downsample_points_rosmsg);
    downsample_points_pub.publish(downsample_points_rosmsg);   
    

    // ============================================== outlier radius_outlier_removal
    sensor_msgs::PointCloud2 radiusOutlierRemovalPointsRosMsg;   
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // first filter
    outrem.setInputCloud(downsample_points);
    outrem.setRadiusSearch(setRadiusSearch);
    outrem.setMinNeighborsInRadius(setMinNeighborsInRadius);
    outrem.filter(*radiusOutlierRemovalPoints);
    if(0 == radiusOutlierRemovalPoints->points.size())
    {
      ROS_ERROR("ERROR! radiusOutlierRemovalPoints1 size is 0!");
      cloudFilterSucceed = false;
      //return;
    } 	
    pcl::toROSMsg(*radiusOutlierRemovalPoints, radiusOutlierRemovalPointsRosMsg);
    radiusOutlierRemovalPoints_pub.publish(radiusOutlierRemovalPointsRosMsg);       
    // second filter  
    outrem.setInputCloud(radiusOutlierRemovalPoints);
    outrem.setRadiusSearch(setRadiusSearch2);
    outrem.setMinNeighborsInRadius(setMinNeighborsInRadius2);
    outrem.filter(*radiusOutlierRemovalPoints2);   
    if(0 == radiusOutlierRemovalPoints2->points.size())
    {
      ROS_ERROR("ERROR! radiusOutlierRemovalPoints2 size is 0!");
      cloudFilterSucceed = false;
      //return;
    } 	
    pcl::toROSMsg(*radiusOutlierRemovalPoints2, radiusOutlierRemovalPointsRosMsg);
    radiusOutlierRemovalPoints2_pub.publish(radiusOutlierRemovalPointsRosMsg);      
    ROS_INFO_STREAM("cloudFilterSucceed: " << cloudFilterSucceed);
    ros::Time filterDoneTime = ros::Time::now();
    ROS_INFO_STREAM("filterDoneTime = " << filterDoneTime - beginTime);    


	  // ============================================== cluster and bounding box
    std::vector<Dectected_Obj> objList;
    objList = clusterSegmentBoundingBox(radiusOutlierRemovalPoints2, clusterTolerance, clusterMinSize, clusterMaxSize);
    ROS_WARN_STREAM("object number: " << objList.size());
    ros::Time clusterDoneTime = ros::Time::now();
    ROS_INFO_STREAM("clusterDoneTime = " << clusterDoneTime - filterDoneTime);  

    jsk_recognition_msgs::BoundingBoxArray boundingBoxArray;
    for(size_t i=0; i<objList.size(); i++)
    {
        boundingBoxArray.boxes.push_back(objList[i].boundingBox_);
    }
    // boundingBoxArray.header = cloud->header;
    boundingBoxArray.header.seq = cloud->header.seq;
    boundingBoxArray.header.frame_id = cloud->header.frame_id;    
    boundingBox_pub.publish(boundingBoxArray);

    // pub markers:  centroid of clustered cloud(equal to the "centroid of cloud" computed from compute3DCentroid().)
    visualization_msgs::MarkerArray boundingBoxCentroidArray, boundingBoxCenterArray;  
    visualization_msgs::Marker boundingBoxCentroid, boundingBoxCenter;
    // 1. centroid
    boundingBoxCentroid.type = visualization_msgs::Marker::SPHERE;   
    boundingBoxCentroid.action = visualization_msgs::Marker::ADD;
    boundingBoxCentroid.lifetime = ros::Duration(0);
    boundingBoxCentroid.header = boundingBoxArray.header; 
    // boundingBoxCentroid.header.frame_id = cloud->header.frame_id; //"/map";
    // boundingBoxCentroid.header.stamp = cloud->header.stamp; //ros::Time::now();    
    boundingBoxCentroid.scale.x = boundingBoxCentroid.scale.y = boundingBoxCentroid.scale.z = 0.05;
    boundingBoxCentroid.color.r = 1.0;
    boundingBoxCentroid.color.g = 1.0;
    boundingBoxCentroid.color.a = 0.8;    
    boundingBoxCentroid.pose.orientation.w = 1.0; 
    // 2. center
    boundingBoxCenter.type = visualization_msgs::Marker::SPHERE;   
    boundingBoxCenter.action = visualization_msgs::Marker::ADD;
    boundingBoxCenter.lifetime = ros::Duration(0);
    boundingBoxCenter.header = boundingBoxArray.header; 
    // boundingBoxCenter.header.frame_id = cloud->header.frame_id; //"/map";
    // boundingBoxCenter.header.stamp = cloud->header.stamp; //ros::Time::now();    
    boundingBoxCenter.scale.x = boundingBoxCenter.scale.y = boundingBoxCenter.scale.z = 0.05; 
    boundingBoxCenter.color.g = 1.0;
    boundingBoxCenter.color.a = 0.8;    
    boundingBoxCenter.pose.orientation.w = 1.0;   

    int markerID = 0;
    boundingBoxCentroidArray.markers.clear();
    boundingBoxCenterArray.markers.clear();  
    for(size_t i=0; i<objList.size(); i++) 
    {
        boundingBoxCentroid.id = markerID;
        boundingBoxCentroid.pose.position.x = objList[i].centroid_.x; // in point cloud(camera) frame.
        boundingBoxCentroid.pose.position.y = objList[i].centroid_.y; 
        boundingBoxCentroid.pose.position.z = objList[i].centroid_.z;    
        boundingBoxCentroidArray.markers.push_back(boundingBoxCentroid);
        boundingBoxCentroid.pose.position.x = objList[i].centroid_.z;  // in base_link frame.
        boundingBoxCentroid.pose.position.y = -objList[i].centroid_.x; 
        boundingBoxCentroid.pose.position.z = -objList[i].centroid_.y;           
        boundingBoxCenter.id = markerID;
        boundingBoxCenter.pose.position.x = objList[i].center_.x(); // in point cloud(camera) frame.
        boundingBoxCenter.pose.position.y = objList[i].center_.y(); 
        boundingBoxCenter.pose.position.z = objList[i].center_.z();    
        boundingBoxCenterArray.markers.push_back(boundingBoxCenter);    
        boundingBoxCenter.pose.position.x = objList[i].center_.z(); // in base_link frame.
        boundingBoxCenter.pose.position.y = -objList[i].center_.x(); 
        boundingBoxCenter.pose.position.z = -objList[i].center_.y();   
        // std::cout<<"\nBBX center(cloud_frame)=\n"<<objList[i].center_<<std::endl;
        // std::cout<<"BBX centroid(cloud_frame)=\n"<<objList[i].centroid_<<std::endl;                
        // std::cout<<"BBX center(base_frame)=\n"<<boundingBoxCenter.pose.position<<std::endl;
        // std::cout<<"BBX centroid(base_frame)=\n"<<boundingBoxCentroid.pose.position<<std::endl;                  

        ++markerID;  
    }  
    boundingBoxCentroidMarker_pub.publish(boundingBoxCentroidArray);
    boundingBoxCenterMarker_pub.publish(boundingBoxCenterArray);  


	// ============================================== icp    
    *incloud = *radiusOutlierRemovalPoints2;
    if(!incloud->is_dense)
    {
        ROS_WARN("input cloud is_dense=false!");
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*incloud, *incloudFiltered, indices);
    }
    else
    {
        *incloudFiltered = *incloud;
    }

    // K1 & K2
    if(!isInitialized)
    {
        pcl::copyPointCloud(*incloudFiltered, *source);

        odomTK2 = odomTCurrent;
        odomRK2 = odomRCurrent;

        bbxTK2[0] = boundingBoxCenter.pose.position.x;
        bbxTK2[1] = boundingBoxCenter.pose.position.y;
        bbxTK2[2] = boundingBoxCenter.pose.position.z;

        isInitialized = true;
        return;
    }

    bbxTK1 = bbxTK2;
    bbxTK2[0] = boundingBoxCenter.pose.position.x;
    bbxTK2[1] = boundingBoxCenter.pose.position.y;
    bbxTK2[2] = boundingBoxCenter.pose.position.z;

    // bbx variation
    std_msgs::Float32MultiArray bbxVariation;
    bbxVariation.data.push_back(bbxTK2[0] - bbxTK1[0]);
    bbxVariation.data.push_back(bbxTK2[1] - bbxTK1[1]);
    bbxVariation.data.push_back(bbxTK2[2] - bbxTK1[2]);
    bbxVariation_pub.publish(bbxVariation);    
    std::cout << "\nBBX position variation=\n" << bbxVariation.data[0] << "  " << bbxVariation.data[1] << "  " << bbxVariation.data[2] << std::endl;

    // pose initialization
    if(initialized == false)
    {
        std::cout<<"\npose initialization......"<<std::endl;
        if(initialization(bbxVariation, bbxTK2))
        {
          initialized = true;
          initialPose = pMean;
          ROS_WARN_STREAM("initial pose =\n" << initialPose);
          std::cout<<"pose initialized succeed!\n"<<std::endl;
        }
        else
        {
          return;
        }
    }


    //std_msgs::Float32MultiArray pk2p0_Msg;
    geometry_msgs::PoseStamped pk2p0_Msg;    
    Eigen::Vector3f pk2p0;
    pk2p0[0] = boundingBoxCenter.pose.position.x - initialPose[0];
    pk2p0[1] = boundingBoxCenter.pose.position.y - initialPose[1];
    pk2p0[2] = boundingBoxCenter.pose.position.z - initialPose[2];   
    
    // pk2p0_Msg.data.push_back(pk2p0[0]);
    // pk2p0_Msg.data.push_back(pk2p0[1]);
    // pk2p0_Msg.data.push_back(pk2p0[2]);   
    // pk2p0_Msg.data.push_back(ros::Time::now().toSec());
    // double time = cloud->header.stamp*0.000001;
    // double del=ros::Time::now().toSec()-cloud->header.stamp;
    // float df = float(del);
    //pk2p0_Msg.header.stamp = ros::Time::now();
    pk2p0_Msg.header.stamp = cloudSubTime;    
    pk2p0_Msg.pose.position.x = pk2p0[0];
    pk2p0_Msg.pose.position.y = pk2p0[1];
    pk2p0_Msg.pose.position.z = pk2p0[2]; 

    std_msgs::Float32MultiArray odomCloud;    
    odomCloud.data.push_back(odomTime);
    odomCloud.data.push_back(odomX);
    odomCloud.data.push_back(odomY);
    odomCloud.data.push_back(odomZ);
    odomCloud.data.push_back(cloud->header.stamp);
    odomCloud.data.push_back(pk2p0[0]);
    odomCloud.data.push_back(pk2p0[1]);
    odomCloud.data.push_back(pk2p0[2]);
    // pk2p0_pub.publish(odomCloud); 
    cloudDone = true;
    pk2p0_pub.publish(pk2p0_Msg); 
/*     std::cout<<"ros time = "<<ros::Time::now()<<std::endl;
    std::cout<<"cloud header time = "<<cloud->header.stamp<<std::endl; */
    ROS_ERROR_STREAM("cloud time = "<<ros::Time::now().toSec()-cloud->header.stamp*0.000001);
    ROS_ERROR_STREAM("total time = "<<ros::Time::now().toSec()-beginTime.toSec());
    std::cout<<"cloud subscribe time = "<<cloud->header.stamp*0.000001<<std::endl;
    std::cout<<"cloud process done time = "<<ros::Time::now()<<std::endl;

    std::cout << "*************cloud process done!***********************\n" << std::endl;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "republish");
    ros::NodeHandle node("~");

    read_parameters();

    cloud_pub = node.advertise<sensor_msgs::PointCloud2>("cloud", 5);
    odom_pub = node.advertise<nav_msgs::Odometry>("odom", 5);
    passthroughfilter_points_pub = node.advertise<sensor_msgs::PointCloud2>("passthroughfilter_points", 1);
    downsample_points_pub = node.advertise<sensor_msgs::PointCloud2>("downsample_points", 1);  
    radiusOutlierRemovalPoints_pub  = node.advertise<sensor_msgs::PointCloud2>("radius_outlier_removal_points", 1);
    radiusOutlierRemovalPoints2_pub  = node.advertise<sensor_msgs::PointCloud2>("radius_outlier_removal_points2", 1);  
    referenceCloud_pub = node.advertise<sensor_msgs::PointCloud2>("reference_cloud", 10);
    readingCloud_pub = node.advertise<sensor_msgs::PointCloud2>("reading_cloud", 10);    
    alignedCloud_pub = node.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 10);    


    boundingBoxCentroidMarker_pub = node.advertise<visualization_msgs::MarkerArray>("bounding_box_centroid", 10); 
    boundingBoxCenterMarker_pub = node.advertise<visualization_msgs::MarkerArray>("bounding_box_center", 10);     
    boundingBox_pub = node.advertise<jsk_recognition_msgs::BoundingBoxArray>("bounding_box", 10);    
    odomVariation_pub = node.advertise<std_msgs::Float32MultiArray>("odom_variation", 10);
    icpPoseVariation_pub = node.advertise<std_msgs::Float32MultiArray>("icp_pose_variation", 10);
    bbxVariation_pub = node.advertise<std_msgs::Float32MultiArray>("bbx_variation", 10);
    fusedVariation_pub = node.advertise<std_msgs::Float32MultiArray>("fused_variation", 10);
    poseVariationDiff_pub = node.advertise<std_msgs::Float32MultiArray>("pose_variation_diff", 10);
    // pk2p0_pub = node.advertise<std_msgs::Float32MultiArray>("pk2p0", 20);
    pk2p0_pub = node.advertise<geometry_msgs::PoseStamped>("pk2p0", 20);




    std::string cloudTopic=argv[1], odomTopic=argv[2];
    std::cout << "argv[0]=" << argv[0] << "  argv[1]=" << argv[1] << "  argv[2]=" << argv[2] << std::endl;

    ros::Subscriber cloud_sub = node.subscribe("/bluerov2h_follower/depth/points2", 20, &callback);
    //ros::Subscriber odom_sub = node.subscribe("/bluerov2h_follower/pose_gt", 40, &odomCallback);
/*     message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(node, cloudTopic, 40);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(node, odomTopic, 40);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> mySyncPolicy;
    message_filters::Synchronizer<mySyncPolicy> sync(mySyncPolicy(40), cloud_sub, odom_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2)); */

    // block方式
    // ros::spin();

    // 循环方式
    ros::Rate rate(100);
    while(ros::ok())
    {
        //std::cout << "main loop......" << std::endl;

        ros::spinOnce();
        rate.sleep();  

        if(newCloudFlag)
        {
          std::cout << "newCloudFlag=1" << std::endl;
          cloudMutex.lock();
          cloudFun();
          newCloudFlag = false;
          cloudMutex.unlock();
        }      
    }

    // 多线程方式
/*     ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown(); */

    //std::cout << "" << std::endl;

    return 0;
}