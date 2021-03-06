// author 
// version 

#include "CloudDetection.hpp"
#include <pcl/io/pcd_io.h>

using namespace std;

namespace bluerov_detection_tracking
{
    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloudFiltered(new PointCloud);
    PointCloudRGB::Ptr cloudRGB(new PointCloudRGB);

    // PointCloud::Ptr referenceCloud(new PointCloud);
    // PointCloud::Ptr source(new PointCloud);   
    // PointCloud::Ptr alignedCloud(new PointCloud);    

    // gloable variables.
    nav_msgs::Odometry odomLeader, odomLeader2, initialOdom; // leader odom.
    nav_msgs::Odometry odomFollower, odomFollower2; // follower odom.
    VectorXd bias_BBX2Odom = VectorXd(3);
    geometry_msgs::PointStamped bbxCloud, bbxOdom;    
    geometry_msgs::TransformStamped T_c2o;
    // Vector3d initialPoseOdom, icpOdom;

    // 定位数据获取的时间点
    double localizedTime;

    // 允许frustum范围动态调整的标志
    bool adaptiveFrustum;


    // CloudDetection::CloudDetection(ros::NodeHandle& node)
    //     : nl(node){}
    CloudDetection::CloudDetection(){}

    CloudDetection::~CloudDetection(){
      // nl.shutdown();
    }



    bool CloudDetection::readParameters(const ros::NodeHandle& node)
    {
      ROS_INFO("Loading parameters......");

      //! debug on/off.
      if(!ros::param::get("~debug_publish", debug_publish_)){
        ROS_WARN_STREAM("could not get debug_publish from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~debug_initialize", debug_initialize_)){
        ROS_WARN_STREAM("could not get debug_initialize from parameterserver!");   
        return false;
      }      

      if(!ros::param::get("~useThread", useThread_)){
        ROS_WARN_STREAM("could not get useThread from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~useNED", useNED_)){
        ROS_WARN_STREAM("could not get useNED from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~savePose", savePose_)){
        ROS_WARN_STREAM("could not get savePose from parameterserver!");   
        return false;
      }
      
      if(!ros::param::get("~useKF", useKF_)){
        ROS_WARN_STREAM("could not get useKF from parameterserver!");   
        return false;
      }      
      //! ground truth.
      if(!ros::param::get("~hasGT", hasGT_)){
        ROS_WARN_STREAM("could not get hasGT from parameterserver!");   
        return false;
      }      


      // frequency.
      if(!ros::param::get("~frequency", frequency_)){
        ROS_WARN_STREAM("could not get frequency from parameterserver!");   
        return false;
      }

      // topic name
      if(!ros::param::get("~Topics/cloudTopic", cloudTopic_)){
        ROS_WARN_STREAM("could not get cloudTopic from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~Topics/rectTopic", rectTopic_)){
        ROS_WARN_STREAM("could not get rectTopic from parameterserver!");   
        return false;
      }
      

      // if(!ros::param::get("~Topics/odomLeaderTopic", odomLeaderTopic_)){
      //   ROS_WARN_STREAM("could not get odomLeaderTopic from parameterserver!");  
      //   return false;
      // }

      // if(!ros::param::get("~Topics/odomFollowerTopic", odomFollowerTopic_)){
      //   ROS_WARN_STREAM("could not get odomFollowerTopic from parameterserver!");  
      //   return false;
      // }

      //! object model.
      if(!ros::param::get("~objectModel/useModelCheck", objectModel.useModelCheck)){
        ROS_WARN_STREAM("could not get objectModel.useModelCheck from parameterserver!");  
        return false;
      }   

      if(!ros::param::get("~objectModel/length", objectModel.length)){
        ROS_WARN_STREAM("could not get objectModel.length from parameterserver!");  
        return false;
      }     

      if(!ros::param::get("~objectModel/width", objectModel.width)){
        ROS_WARN_STREAM("could not get objectModel.width from parameterserver!");  
        return false;
      }         

      if(!ros::param::get("~objectModel/height", objectModel.height)){
        ROS_WARN_STREAM("could not get objectModel.height from parameterserver!");  
        return false;
      }        

      if(!ros::param::get("~objectModel/number", objectModel.cloudSize)){
        ROS_WARN_STREAM("could not get objectModel.cloudSize from parameterserver!");  
        return false;
      }        
      if(!ros::param::get("~objectModel/nFactor", objectModel.nFactor)){
        ROS_WARN_STREAM("could not get objectModel.nFactor from parameterserver!");  
        return false;
      }      

      if(!ros::param::get("~objectModel/sFactor_min", objectModel.sFactor_min)){
        ROS_WARN_STREAM("could not get objectModel.sFactor_min from parameterserver!");  
        return false;
      }      
      if(!ros::param::get("~objectModel/sFactor_max", objectModel.sFactor_max)){
        ROS_WARN_STREAM("could not get objectModel.sFactor_max from parameterserver!");  
        return false;
      }             
      if(!ros::param::get("~objectModel/pFactor", objectModel.pFactor)){
        ROS_WARN_STREAM("could not get objectModel.pFactor from parameterserver!");  
        return false;
      }                 

      //! calibration.
      if(!ros::param::get("~calibration/kx", calibration_.kx)){
        ROS_WARN_STREAM("could not get kx from parameterserver!");    
        return false;
      }
      if(!ros::param::get("~calibration/ky", calibration_.ky)){
        ROS_WARN_STREAM("could not get ky from parameterserver!");    
        return false;
      }
      if(!ros::param::get("~calibration/kz", calibration_.kz)){
        ROS_WARN_STREAM("could not get kz from parameterserver!");    
        return false;
      }

      //! pointcloud filter on/off.
      if(!ros::param::get("~pointcloudFilter/debug", cloudFilter.debug_)){
        ROS_WARN_STREAM("could not get debug from parameterserver!");    
        return false;
      }

      if(!ros::param::get("~pointcloudFilter/passthroughFilter", cloudFilter.passthroughFilter_)){
        ROS_WARN_STREAM("could not get passthroughFilter from parameterserver!");    
        return false;
      }

      if(!ros::param::get("~pointcloudFilter/removeGround", cloudFilter.removeGround_)){
        ROS_WARN_STREAM("could not get removeGround from parameterserver!");    
        return false;
      }

      if(!ros::param::get("~pointcloudFilter/voxelDownsample", cloudFilter.voxelDownsample_)){
        ROS_WARN_STREAM("could not get voxelDownsample from parameterserver!");    
        return false;
      }

      if(!ros::param::get("~pointcloudFilter/outlierRemove", cloudFilter.outlierRemove_)){
        ROS_WARN_STREAM("could not get outlierRemove from parameterserver!");    
        return false;
      }      

      // passthrough filter parameters
      if(!ros::param::get("~passthrough_filter/useDynamicROI", useDynamicROI_)){
        ROS_WARN_STREAM("could not get useDynamicROI from parameterserver!");    
        return false;
      }      
      if(!ros::param::get("~passthrough_filter/setFilterLimits_X1", cloudFilter.setFilterLimits_X1_)){
        ROS_WARN_STREAM("could not get setFilterLimits_X1 from parameterserver!");    
        return false;
      }
      
      if(!ros::param::get("~passthrough_filter/setFilterLimits_X2", cloudFilter.setFilterLimits_X2_)){
        ROS_WARN_STREAM("could not get setFilterLimits_X2 from parameterserver!");  
        return false;
      }
      
      if(!ros::param::get("~passthrough_filter/setFilterLimits_Y1", cloudFilter.setFilterLimits_Y1_)){
        ROS_WARN_STREAM("could not get setFilterLimits_Y1 from parameterserver!");    
        return false;
      }
      
      if(!ros::param::get("~passthrough_filter/setFilterLimits_Y2", cloudFilter.setFilterLimits_Y2_)){
        ROS_WARN_STREAM("could not get setFilterLimits_Y2 from parameterserver!");   
        return false;
      }
      
      if(!ros::param::get("~passthrough_filter/setFilterLimits_Z1", cloudFilter.setFilterLimits_Z1_)){
        ROS_WARN_STREAM("could not get setFilterLimits_Z1 from parameterserver!");    
        return false;
      }
      
      if(!ros::param::get("~passthrough_filter/setFilterLimits_Z2", cloudFilter.setFilterLimits_Z2_)){
        ROS_WARN_STREAM("could not get setFilterLimits_Z2 from parameterserver!");
        return false;        
      }

      // downsample filter parameters
      if(!ros::param::get("~downsample/setLeafSize", cloudFilter.setLeafSize_)){
        ROS_WARN_STREAM("could not get setLeafSize_ from parameterserver!");   
        return false;
      }

      //! RadiusOutlierRemoval
      if(!ros::param::get("~OutlierRemoval/setRadiusSearch", cloudFilter.setRadiusSearch_)){
        ROS_WARN_STREAM("could not get setRadiusSearch from parameterserver!"); 
        return false;
      }
      
      if(!ros::param::get("~OutlierRemoval/setMinNeighborsInRadius", cloudFilter.setMinNeighborsInRadius_)){
        ROS_WARN_STREAM("could not get setMinNeighborsInRadius from parameterserver!");     
        return false;
      }

      //! StatisticalOutlierRemoval
      if(!ros::param::get("~OutlierRemoval/setMeanK", cloudFilter.setMeanK_)){
        ROS_WARN_STREAM("could not get setMeanK from parameterserver!"); 
        return false;
      }
      
      if(!ros::param::get("~OutlierRemoval/setStddevMulThresh", cloudFilter.setStddevMulThresh_)){
        ROS_WARN_STREAM("could not get setStddevMulThresh from parameterserver!");     
        return false;
      }      
      
      //! initialization. 
      if(!ros::param::get("~initialization/consecutiveTinyFluctuationTimes", initialCalibrationPara.consecutiveTinyFluctuationTimes)){
        ROS_WARN_STREAM("could not get consecutiveTinyFluctuationTimes from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~initialization/poseFluctuationX", initialCalibrationPara.poseFluctuationX)){
        ROS_WARN_STREAM("could not get poseFluctuationX from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~initialization/poseFluctuationY", initialCalibrationPara.poseFluctuationY)){
        ROS_WARN_STREAM("could not get poseFluctuationY from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~initialization/poseFluctuationZ", initialCalibrationPara.poseFluctuationZ)){
        ROS_WARN_STREAM("could not get poseFluctuationZ from parameterserver!");   
        return false;
      }                  

      // ICP 
      if(!ros::param::get("~ICP/icpMethod", icpMethod_)){
        ROS_WARN_STREAM("could not get icpMethod from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~ICP/setMaximumIterations", setMaximumIterations_)){
        ROS_WARN_STREAM("could not get setMaximumIterations from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~ICP/setTransformationEpsilon", setTransformationEpsilon_)){
        ROS_WARN_STREAM("could not get setTransformationEpsilon from parameterserver!");                   
        return false;
      }

      if(!ros::param::get("~ICP/setMaxCorrespondenceDistance", setMaxCorrespondenceDistance_)){
        ROS_WARN_STREAM("could not get setMaxCorrespondenceDistance from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~ICP/setEuclideanFitnessEpsilon", setEuclideanFitnessEpsilon_)){
        ROS_WARN_STREAM("could not get setEuclideanFitnessEpsilon from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~ICP/setRANSACIterations", setRANSACIterations_)){
        ROS_WARN_STREAM("could not get setRANSACIterations from parameterserver!");   
        return false;
      }

      // cluster
      if(!ros::param::get("~cluster/clusterTolerance", clusterTolerance_)){
        ROS_WARN_STREAM("could not get clusterTolerance from parameterserver!"); 
        return false;
      }

      if(!ros::param::get("~cluster/clusterMinSize", clusterMinSize_)){
        ROS_WARN_STREAM("could not get clusterMinSize from parameterserver!");   
        return false;
      }

      if(!ros::param::get("~cluster/clusterMaxSize", clusterMaxSize_)){
        ROS_WARN_STREAM("could not get clusterMaxSize from parameterserver!");         
        return false;
      }
      
      // LOG file directory
      if(!ros::param::get("~saveCloudDir", saveCloudDir_)){
        ROS_WARN_STREAM("could not get saveCloudDir from parameterserver!");         
        return false;
      }

      // frustum
      if(!ros::param::get("~frustum/depthMin", depthMin_)){
        ROS_WARN_STREAM("could not get depthMin from parameterserver!");         
        return false;
      }     

      if(!ros::param::get("~frustum/depthMax", depthMax_)){
        ROS_WARN_STREAM("could not get depthMax from parameterserver!");         
        return false;
      }       
      if(!ros::param::get("~frustum/adaptive", adaptiveFrustum)){
        ROS_WARN_STREAM("could not get adaptiveFrustum from parameterserver!");         
        return false;
      }    
          

      //
      if(!ros::param::get("~cameraIntrinsic/fx", cameraIntrinsic.fx)){
        ROS_WARN_STREAM("could not get fx from parameterserver!");         
        return false;
      }       
      
      if(!ros::param::get("~cameraIntrinsic/fy", cameraIntrinsic.fy)){
        ROS_WARN_STREAM("could not get fy from parameterserver!");         
        return false;
      }    

      if(!ros::param::get("~cameraIntrinsic/cx", cameraIntrinsic.cx)){
        ROS_WARN_STREAM("could not get cx from parameterserver!");         
        return false;
      }    

      if(!ros::param::get("~cameraIntrinsic/cy", cameraIntrinsic.cy)){
        ROS_WARN_STREAM("could not get cy from parameterserver!");         
        return false;
      }    

      //
      if(!ros::param::get("~KalmanFilter/axNoise", kalmanFilaterPara.axNoise)){
        ROS_WARN_STREAM("could not get axNoise from parameterserver!");         
        return false;
      }  
      if(!ros::param::get("~KalmanFilter/ayNoise", kalmanFilaterPara.ayNoise)){
        ROS_WARN_STREAM("could not get ayNoise from parameterserver!");         
        return false;
      }  
      if(!ros::param::get("~KalmanFilter/azNoise", kalmanFilaterPara.azNoise)){
        ROS_WARN_STREAM("could not get azNoise from parameterserver!");         
        return false;
      }        

      if(!ros::param::get("~KalmanFilter/xNoise", kalmanFilaterPara.xNoise)){
        ROS_WARN_STREAM("could not get xNoise from parameterserver!");         
        return false;
      }  
      if(!ros::param::get("~KalmanFilter/yNoise", kalmanFilaterPara.yNoise)){
        ROS_WARN_STREAM("could not get yNoise from parameterserver!");         
        return false;
      }  
      if(!ros::param::get("~KalmanFilter/zNoise", kalmanFilaterPara.zNoise)){
        ROS_WARN_STREAM("could not get zNoise from parameterserver!");         
        return false;
      }        

      if(!ros::param::get("~poseJumpBiasThreshold/x", poseJumpBiasThreshold_[0])){
        ROS_WARN_STREAM("could not get poseJumpBiasThreshold x from parameterserver!");         
        return false;
      }      
      if(!ros::param::get("~poseJumpBiasThreshold/y", poseJumpBiasThreshold_[1])){
        ROS_WARN_STREAM("could not get poseJumpBiasThreshold y from parameterserver!");         
        return false;
      }     
      if(!ros::param::get("~poseJumpBiasThreshold/z", poseJumpBiasThreshold_[2])){
        ROS_WARN_STREAM("could not get poseJumpBiasThreshold z from parameterserver!");         
        return false;
      }     

      return true;
    }


    bool CloudDetection::initialize(const ros::NodeHandle& n)
    {
      ROS_INFO("Initialization ......");

      // 结构体初始化 --------------------------------------------

      //! InitialCalibrationInfo
      initialCalibrationInfo.trapezoidMinDepth = 0.5;
      initialCalibrationInfo.trapezoidMaxDepth = 3.0;   
      initialCalibrationInfo.referenceCloud.reset(new PointCloud);
      initialCalibrationInfo.p0 << 0., 0., 0.;     
      initialCalibrationInfo.initialized = false;
      initialCalibrationInfo.poseInitialized = false;
      initialCalibrationInfo.trapezoidInitialized = false;
      initialCalibrationInfo.cloudInitialized = false;
      
      //! initialCalibrationPara
      initialCalibrationPara.poseFluctuationX = 0.03;
      initialCalibrationPara.poseFluctuationY = 0.03;
      initialCalibrationPara.poseFluctuationZ = 0.03;
      initialCalibrationPara.consecutiveTinyFluctuationTimes = 5;

      // ObjectModel
      objectModel.useModelCheck = false;      
      objectModel.length = 0.4;
      objectModel.width = 0.4;
      objectModel.height = 0.5;
      objectModel.cloudSize = 500;
      objectModel.nFactor = 0.5;
      objectModel.sFactor_min = 0.5;
      objectModel.sFactor_max = 1.5;    
      objectModel.pFactor = 0.5;

      //! debug
      debug_publish_ = true;
      debug_initialize_ = false;
      debug_kf_ = false;

      firstBBX_ = true;      
      worldFrame_ = false;
      useRelativePose_ = false;
      roiL_ = roiW_ = roiH_ = 0.5;
      useDynamicROI_ = false;

      //
      cameraIntrinsic.fx = 679.2294311523438;
      cameraIntrinsic.fy = 679.2294311523438;
      cameraIntrinsic.cx = 620.7433471679688;
      cameraIntrinsic.cy = 350.60980224609375;

      poseJumpBiasThreshold_[0] = 0.2;
      poseJumpBiasThreshold_[1] = 0.2;
      poseJumpBiasThreshold_[2] = 0.2;      


      //! Kalman Filter参数初始化
      // 运动模型：匀速运动模型，
      // 状态空间变量：3D空间的位置和线速度
      kf.x_ = VectorXd(6);  // create a 6D(x,y,z, vx,vy,vz) state vector, we don't know yet the values of the x state
      // locked_pose坐标系下，初始位置都为0
      kf.x_ <<  0, 
                0, 
                0,
                0, 
                0,
                0;          

      kalmanFilaterPara.axNoise = 9;
      kalmanFilaterPara.ayNoise = 25;
      kalmanFilaterPara.azNoise = 25;
      kalmanFilaterPara.xNoise = 0.3;
      kalmanFilaterPara.yNoise = 0.1;
      kalmanFilaterPara.zNoise = 0.1;

      // the initial transition matrix F_
      kf.F_ = MatrixXd(6, 6);
      kf.F_ <<  1, 0, 0, 1, 0, 0,
                0, 1, 0, 0, 1, 0,
                0, 0, 1, 0, 0, 1,
                0, 0, 0, 1, 0, 0,    
                0, 0, 0, 0, 1, 0,      
                0, 0, 0, 0, 0, 1;      

      // state covariance matrix P
      kf.P_ = MatrixXd(6, 6);
      kf.P_ <<  1000, 0, 0, 0, 0, 0,
                0, 1000, 0, 0, 0, 0,
                0, 0, 1000, 0, 0, 0,
                0, 0, 0, 1000, 0, 0,
                0, 0, 0, 0, 1000, 0,                
                0, 0, 0, 0, 0, 1000;    
                           
      // measurement matrix
      kf.H_ = MatrixXd(3, 6);
      kf.H_ <<  1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0;   

      kf.Q_ = MatrixXd(6, 6);    

      // set the measurement covariance matrix R
      kf.R_ = MatrixXd(3,3);

      // previousTime_ = 0;  
      previousTime_ = ros::Time::now().toSec();


      // 从服务器加载参数 --------------------------------------------
      if(!readParameters(n)){
        ROS_ERROR("Failed to load parameters.");
        return false;
      };


      // 全局变量初始化 --------------------------------------------
      consecutiveTinyFluctuationPoseSum_ << 0., 0., 0.; // 初始化为0,防止出现很大的随机数导致程序崩掉！     
      consecutiveTinyFluctuationCout_ = 0; // 必须初始化，否则出现随机很大的数导致计算初始化位置始终为0！

      cloudHeader_.frame_id = "zed_left_camera_frame";
      

      ROS_INFO("Done.");


      // 主处理函数 --------------------------------------------
      if(!registerCallbacks(n)){
        ROS_ERROR("Failed to register callbacks.");
        return false;
      }

      return true;
    }


    bool CloudDetection::registerCallbacks(const ros::NodeHandle& n)
    {
      ros::NodeHandle nl(n);

      // Timer 
      cloudTimer_ = nl.createTimer(ros::Duration(1/frequency_), &CloudDetection::cloudProcess, this);  


      //！subscriber 
      cloud_sub = nl.subscribe(cloudTopic_, 20, &CloudDetection::cloudCallback, this);
      cascadeDetectionResult_sub = nl.subscribe(rectTopic_, 20, &CloudDetection::cascadeDetectionResultCallback, this);


      //！publisher 
      incloudFiltered_pub = nl.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 20);
      // referenceCloud_pub = nl.advertise<sensor_msgs::PointCloud2>("reference_cloud", 20);
      // readingCloud_pub = nl.advertise<sensor_msgs::PointCloud2>("reading_cloud", 20);    
      // alignedCloud_pub = nl.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 20);   

      // 发布定位结果：目标几何中心
      bbxCenterMarker_pub = nl.advertise<visualization_msgs::Marker>("bbx_center", 20);   
      fusedMarker_pub = nl.advertise<visualization_msgs::Marker>("fused_center", 20);

      // 发布定位结果：bbx
      referenceBBX_pub = nl.advertise<jsk_recognition_msgs::BoundingBox>("reference_bbx", 20);
      bbx_pub = nl.advertise<jsk_recognition_msgs::BoundingBox>("bbx", 20);    
      ROI_pub = nl.advertise<jsk_recognition_msgs::BoundingBox>("roi", 20);  
      // 发布定位结果
      pose_pub = nl.advertise<geometry_msgs::PoseStamped>("bbxpose", 20);  
      // bbxpose_ned_pub = nl.advertise<geometry_msgs::PoseStamped>("bbxpose_ned", 10);          
      fusedpose_pub = nl.advertise<geometry_msgs::PoseStamped>("fusedpose", 20);  
      // fusedpose_ned_pub = nl.advertise<geometry_msgs::PoseStamped>("fusedpose_ned", 10);          
      pose_ned_pub = nl.advertise<geometry_msgs::PoseStamped>("pose_ned", 20);             
      // 发布目标轨迹
      lockedFramePath_pub = nl.advertise<visualization_msgs::Marker>("locked_frame_path", 20);
      posePath_pub = nl.advertise<visualization_msgs::Marker>("pose_path", 20);
      nedPath_pub = nl.advertise<visualization_msgs::Marker>("ned_path", 20);
      fusedPath_pub = nl.advertise<visualization_msgs::Marker>("fused_path", 20);
      // 发布锥体ROI
      // frustum_pub = nl.advertise<visualization_msgs::Marker>("frustum", 20);
      trapezoid3D_pub = nl.advertise<visualization_msgs::Marker>("trapezoid3D", 20);      


      //！service 
      saveCloudService = nl.advertiseService("save_cloud", &CloudDetection::saveCloud, this);
      saveLogService = nl.advertiseService("save_log", &CloudDetection::saveLog, this);
      computeRMSEService =nl.advertiseService("compute_rmse", &CloudDetection::computeRMSE, this);      

      return true;        
    }


    bool CloudDetection::poseInitialization(Eigen::Vector3d previousPose, 
                                            Eigen::Vector3d currentPose)
    {
        // 计算位置变化量
        Eigen::Vector3d deltaPose;
        deltaPose = currentPose - previousPose;
        // float delta_X = 0.0, delta_Y = 0.0, delta_Z=0.0;
        // delta_X = delta_P(0);
        // delta_Y = delta_P(1);
        // delta_Z = delta_P(2);  
        if(debug_initialize_) cout<<"Pose delta = "<<endl<<deltaPose<<endl;      

        // 判定位置变化量是否满足阈值条件（小范围内波动）
        bool if_X, if_Y, if_Z;
        if_X = if_Y = if_Z = false;
        if(fabs(deltaPose(0)) < initialCalibrationPara.poseFluctuationX)
          if_X = true;          
        if(fabs(deltaPose(1)) < initialCalibrationPara.poseFluctuationY)
          if_Y = true;
        if(fabs(deltaPose(2)) < initialCalibrationPara.poseFluctuationZ)
          if_Z = true;

        // 累加位置波动变化量
        if(if_X==true && if_Y==true && if_Z==true)
        {
          ++consecutiveTinyFluctuationCout_;
          consecutiveTinyFluctuationPoseSum_[0] = consecutiveTinyFluctuationPoseSum_[0] + currentPose[0];
          consecutiveTinyFluctuationPoseSum_[1] = consecutiveTinyFluctuationPoseSum_[1] + currentPose[1];
          consecutiveTinyFluctuationPoseSum_[2] = consecutiveTinyFluctuationPoseSum_[2] + currentPose[2];
          if(debug_initialize_) cout<<"consecutiveTinyFluctuationPoseSum_ = "<<endl<<consecutiveTinyFluctuationPoseSum_<<endl;
        }
        else
        {
          consecutiveTinyFluctuationCout_ = 0;
          consecutiveTinyFluctuationPoseSum_[0] = 0.0;
          consecutiveTinyFluctuationPoseSum_[1] = 0.0;
          consecutiveTinyFluctuationPoseSum_[2] = 0.0;

          return false;
        }

        // 只有当“连续”测量位置点都在指定的阈值范围内波动时，才认为初始化成功。        
        if(consecutiveTinyFluctuationCout_ >= initialCalibrationPara.consecutiveTinyFluctuationTimes)
        { 
          initialCalibrationInfo.p0[0] = consecutiveTinyFluctuationPoseSum_[0]/consecutiveTinyFluctuationCout_;
          initialCalibrationInfo.p0[1] = consecutiveTinyFluctuationPoseSum_[1]/consecutiveTinyFluctuationCout_;
          initialCalibrationInfo.p0[2] = consecutiveTinyFluctuationPoseSum_[2]/consecutiveTinyFluctuationCout_;
          if(debug_initialize_){
            cout<<"consecutiveTinyFluctuationCout_ = "<<consecutiveTinyFluctuationCout_<<endl;
            cout<<"consecutiveTinyFluctuationPoseSum_ = "<<endl<<consecutiveTinyFluctuationPoseSum_<<endl;
            cout<<"initialCalibrationInfo.p0 = "<<endl<<initialCalibrationInfo.p0<<endl;
          }          
          return true; 
        }

        return false;
    }


    void CloudDetection::cloudProcess(const ros::TimerEvent& event)
    {
      // 只有当同时满足以下2个条件时才进行点云处理：（1）订阅到新点云；（2）订阅到2D目标检测结果
      if(newCloudFlag && cascadeDetectSuccess){
        std::cout<<std::fixed<<std::setprecision(3);    

        cloudMutex.lock();

        std::cout<<"\n===================== cloud processing =========================" << std::endl;
        ros::Time beginTime = ros::Time::now();		
        pcl_conversions::fromPCL(cloud->header, cloudHeader_);
        std::cout<<"Time when cloud subscribed(1) = "<<cloud->header.stamp*1e-6<<std::endl; 

        // ============================================== 点云预处理： 直通滤波，[去除地面]，降采样，移除离散点
        cout<<endl;
        cout<<"Cloud filtering ......"<<endl;

        if(!cloudFilter.Filter(cloud, cloudFiltered)){
          ROS_ERROR("Cloud filtering failted.");
          cloudMutex.unlock();
          return;
        }

        if(!cloudFilter.convexHullFilter(cloudFiltered, detectedObject2D.trapezoid3DVertex,cloudFiltered)){
          cloudMutex.unlock();
          return;          
        }

        // 如果滤波后点云数量太少，则不进行后续处理
        if(cloudFiltered->size()<100){
          ROS_WARN_STREAM("Cloud number after filtered is too small = " << cloudFiltered->size());
          cloudMutex.unlock();
          return;
        }

        // remove NAN from cloud.
        if(!cloudFiltered->is_dense)
        {
            ROS_WARN("input cloud is_dense=false!");
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloudFiltered, *cloudFiltered, indices);
        }

        // 统计时间
        if(debug_time_){
          cout<<"Time used = " << ros::Time::now() - beginTime<<endl;
          cout<<endl; 
        }
 
        //! 发布滤波后的点云，以及直通滤波器ROI到RVIZ显示
        if(debug_publish_){
          cout<<"Initial cloud size = "<<cloud->size()<<"\t"<<"After filtered size = "<<cloudFiltered->size()<<endl;

          //! filtered cloud.
          sensor_msgs::PointCloud2 cloudFilteredRosMsg;
          pcl::toROSMsg(*cloudFiltered, cloudFilteredRosMsg);
          incloudFiltered_pub.publish(cloudFilteredRosMsg);

          //! filter ROI
          jsk_recognition_msgs::BoundingBox roi;
          roi.header = cloudHeader_;
          roiL_ = fabs(cloudFilter.setFilterLimits_X1_ - cloudFilter.setFilterLimits_X2_);
          roiW_ = fabs(cloudFilter.setFilterLimits_Y1_ - cloudFilter.setFilterLimits_Y2_);
          roiH_ = fabs(cloudFilter.setFilterLimits_Z1_ - cloudFilter.setFilterLimits_Z2_);          
          roi.pose.position.x = cloudFilter.setFilterLimits_X1_ + 0.5*roiL_;
          roi.pose.position.y = cloudFilter.setFilterLimits_Y1_ + 0.5*roiW_;
          roi.pose.position.z = cloudFilter.setFilterLimits_Z1_ + 0.5*roiH_;
          roi.dimensions.x = roiL_;
          roi.dimensions.y = roiW_;
          roi.dimensions.z = roiH_;
          ROI_pub.publish(roi);
        }


        // ============================================== 初始化： 计算目标初始位置，保存参考点云
        if(!initialCalibrationInfo.initialized)
        {
          if(!initialCalibrationInfo.poseInitialized){
            cout<<endl;
            cout<<"STEP1. Object initialization......"<<endl;

            // 点云聚类获取目标中心位置
            int clusterNumber = 0;
            clusterNumber = updateBBXThread(clusterTolerance_, clusterMinSize_, clusterMaxSize_);
            if(clusterNumber == 0){
                cloudMutex.unlock();
                return;                 
            };

            if(debug_initialize_){
              cout<<"BBX dimentions, length="<<detectedObjectInfo.bbx.dimensions.x<<
                                    " width="<<detectedObjectInfo.bbx.dimensions.y<<
                                    " height="<<detectedObjectInfo.bbx.dimensions.z<<endl;              
            }

            // （可选步骤）检查当前目标BBX与用户设定的目标BBX的几何模型参数是否一致
            if(objectModel.useModelCheck){
              cout<<"Model dimentions, length="<<objectModel.length<<
                                    " width="<<objectModel.width<<
                                    " height="<<objectModel.height<<
                                    " points numbers="<<objectModel.cloudSize<<endl;                                    

              if(!modelCheck()){
                cloudMutex.unlock();
                return;              
              }
            }

            // 针对第一次检查到目标，要做特殊处理
            if(firstBBX_){
              previousPose_ = detectedObjectInfo.center;
              if(debug_initialize_){
                cout<<"This is the 1st time detected object."<<endl;          
                cout<<"previousPose_ = "<<previousPose_<<endl;
              }
              firstBBX_ = false;
              cloudMutex.unlock();
              return;
            }


            //！ 位置初始化 -------------------------------------------------
            // 调用位置初始化函数
            cout<<endl;
            cout<<"Pose initialization......"<<endl;            
            if(poseInitialization(detectedObjectInfo.center, previousPose_))
            {
                initialCalibrationInfo.poseInitialized = true;

                // 记录机器人初始位置.[为follower的点云坐标系,Z轴朝前]
                initialCalibrationInfo.p0(0) *=   calibration_.kx; 
                initialCalibrationInfo.p0(1) *=   calibration_.ky;
                initialCalibrationInfo.p0(2) *=   calibration_.kz;

                cout<<"Object initial pose p0 = "<<endl<<initialCalibrationInfo.p0<<endl;
            }
            // else
            // {
            //   cloudMutex.unlock();
            //   return;
            // }      
          }
          else // 位置初始化成功
          { 
              cout<<endl;
              cout<<"Reference cloud and bbx initialization......"<<endl;          

              // //！ Kalman Filter初始化 -------------------------------------------------
              // // 点云坐标系
              // kf.x_ <<  initialCalibrationInfo.p0(0), 
              //           initialCalibrationInfo.p0(1), 
              //           initialCalibrationInfo.p0(2),
              //           0, 
              //           0,
              //           0;          
              // cout << "Set Kalman Filter initial state X = " <<endl<<kf.x_<<endl;


              //！ 点云初始化：获取目标初始参考点云 ------------------------------------------------- 

              // 将滤波后目标点云拷贝到全局变量中，作为初始参考点云
              pcl::copyPointCloud(*cloudFiltered, *initialCalibrationInfo.referenceCloud);

              // 保存目标初始点云BBX信息
              initialCalibrationInfo.bbx = detectedObjectInfo.bbx;
              if(debug_initialize_){
                cout<<"Reference BBX info : "<<initialCalibrationInfo.bbx<<endl;
              }
              referenceBBX_pub.publish(initialCalibrationInfo.bbx);
              initialCalibrationInfo.cloudInitialized = true;

              // 更新几何模型参数：根据初始化BBX结果来设置object model参数
              objectModel.length = 1 * initialCalibrationInfo.bbx.dimensions.x;
              objectModel.width = 1 * initialCalibrationInfo.bbx.dimensions.y;
              objectModel.height = 1 * initialCalibrationInfo.bbx.dimensions.z;
              objectModel.cloudSize = 1 * initialCalibrationInfo.referenceCloud->size();

              initialCalibrationInfo.initialized = true;

              cloudMutex.unlock();

              cout<<"Done."<<"\n"<<endl;
              for(int i_ok=0; i_ok<10; ++i_ok){ // 为了能够有充足时间在终端看到已经初始化成功了，多打印几遍...
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"        ------"<<"        |    /"<<endl;
                cout<<"        |     |"<<"       |   /"<<endl;
                cout<<"        |     |"<<"       |  /"<<endl;   
                cout<<"        |     |"<<"       | /"<<endl;   
                cout<<"        |     |"<<"       |  |"<<endl;   
                cout<<"        |     |"<<"       |   |"<<endl;   
                cout<<"        ------"<<"        |    |"<<endl;   
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;
                cout<<"*************************************************************************************"<<endl;   
              }           
          }

          cloudMutex.unlock();
          return;         
        }


        // ============================================== 添加locked_pose frame，为静态TF，根据初始化获取的初始位置确定两者相对位置关系
        // 注意：locked_pose坐标系和NED坐标系原点重合，但三个轴方向不同。locked_pose - 前左上， ned - 前右下
        std::string fatherFrame = "zed_left_camera_frame";
        std::string childFrame = "locked_pose";

        geometry_msgs::PoseStamped lockedPose;
        lockedPose.pose.position.x = initialCalibrationInfo.p0(0);
        lockedPose.pose.position.y = initialCalibrationInfo.p0(1); 
        lockedPose.pose.position.z = initialCalibrationInfo.p0(2);
        lockedPose.pose.orientation.x = 0.;
        lockedPose.pose.orientation.y = 0.;
        lockedPose.pose.orientation.z = 0.;
        lockedPose.pose.orientation.w = 1.;

        utility.sendTF(fatherFrame, childFrame, lockedPose);


        // ============================================== 添加NED frame，为静态TF，根据初始化获取的初始位置确定两者相对位置关系
        // http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28C%2B%2B%29
        // http://wiki.ros.org/tf2/Tutorials

        // ZED TF: map->odom->base_link->zed_left_camera_frame->zed_left_camera_optical_frame     

        if(useNED_){
          // NED frame: x-north, y-east, z-down
          fatherFrame = "zed_left_camera_frame";
          childFrame = "ned";

          geometry_msgs::PoseStamped nedPose;
          geometry_msgs::Point zedCenter2zedLeft;
          // zedCenter2zedLeft.y = 0.06; // baseline/2
          nedPose.pose.position.x = initialCalibrationInfo.p0(0);
          nedPose.pose.position.y = initialCalibrationInfo.p0(1); // + zedCenter2zedLeft.y;
          nedPose.pose.position.z = initialCalibrationInfo.p0(2);
          nedPose.pose.orientation.x = 0.;
          nedPose.pose.orientation.y = 0.;
          nedPose.pose.orientation.z = 0.;
          nedPose.pose.orientation.w = 1.;
          // nedPose.pose.orientation.x = nedPose.pose.orientation.y = nedPose.pose.orientation.z = 0.0;

          utility.sendTF(fatherFrame, childFrame, nedPose);
        }


        // ============================================== 目标检测、定位
        cout<<endl;
        cout<<"object detection ......"<<endl;
        bbxThread_ = std::thread(&CloudDetection::updateBBXThread, this, clusterTolerance_, clusterMinSize_, clusterMaxSize_);   
                    
        // ros::Time threadTime0 = ros::Time::now();   
        bbxThread_.join();    


        // 滤波1：模型检测。根据预定义的目标模型对检测结果进行判断，符合模型则保留，否则放弃
        if(objectModel.useModelCheck){
          if(!modelCheck()){
            ROS_WARN("Detected object cloud points not meet Geometry Model constraint!");
            cout<<"BBX dimentions, length="<<detectedObjectInfo.bbx.dimensions.x<<
                                  " width="<<detectedObjectInfo.bbx.dimensions.y<<
                                  " height="<<detectedObjectInfo.bbx.dimensions.z<<endl;      
            cout<<"MODEL dimentions, length="<<objectModel.length<<
                                  " width="<<objectModel.width<<
                                  " height="<<objectModel.height<<endl;                                              
            cloudMutex.unlock();
            return;              
          }
        }            
        // cout<<"Time used for cloud object detection thread = " << ros::Time::now() - threadTime0<<endl;    


        // 滤波2：跳变检测。前后两帧目标位置比较，超过跳变阈值则使用前一帧检测结果
        Eigen::Vector3d poseJumpBiasValue;
        poseJumpBiasValue[0] = detectedObjectInfo.center(0) * calibration_.kx - previousPose_[0];
        poseJumpBiasValue[1] = detectedObjectInfo.center(1) * calibration_.kx - previousPose_[1];
        poseJumpBiasValue[2] = detectedObjectInfo.center(2) * calibration_.kx - previousPose_[2];
        bool poseJumped = false;
        if(poseJumpBiasValue[0] > poseJumpBiasThreshold_[0]){
          ROS_WARN_STREAM("pose jumped in X: "<<poseJumpBiasValue[0]);
          poseJumped = true;
        }
        if(poseJumpBiasValue[1] > poseJumpBiasThreshold_[1]){
          ROS_WARN_STREAM("pose jumped in Y: "<<poseJumpBiasValue[1]);
          poseJumped = true;
        }    
        if(poseJumpBiasValue[2] > poseJumpBiasThreshold_[2]){
          ROS_WARN_STREAM("pose jumped in Z: "<<poseJumpBiasValue[2]);
          poseJumped = true;
        }            
        if(poseJumped){
          detectedObjectInfo.center(0) = previousPose_[0];
          detectedObjectInfo.center(1) = previousPose_[1];
          detectedObjectInfo.center(2) = previousPose_[2];
        }


        // 更新前一帧目标位置
        previousPose_[0] = detectedObjectInfo.center(0) * calibration_.kx;
        previousPose_[1] = detectedObjectInfo.center(1) * calibration_.ky;
        previousPose_[2] = detectedObjectInfo.center(2) * calibration_.kz;

        // 位置变化量（相对于初始位置)
        // useRelativePose_ = true;
        // if(useRelativePose_){
        //   poseDelta_ = previousPose_ - initialCalibrationInfo.p0;
        //   }            

   
        //目标位置：相对于初始位置
        cout<<endl<<"P0 = "<<endl<<initialCalibrationInfo.p0<<endl;
        cout<<"Object position = "<<endl<<detectedObjectInfo.center<<endl<<endl;
        localizationResult.pose.header = cloudHeader_;
        localizationResult.pose.pose.position.x = detectedObjectInfo.center.x() * calibration_.kx;
        localizationResult.pose.pose.position.y = detectedObjectInfo.center.y() * calibration_.ky;
        localizationResult.pose.pose.position.z = detectedObjectInfo.center.z() * calibration_.kz;

        // 保存定位结果到本地txt文件
        bbxEst.push_back(detectedObjectInfo.center);


        // 目标BBX到RVIZ显示
        if(debug_publish_){

          // 矩形框          
          jsk_recognition_msgs::BoundingBox bbx;
          bbx = detectedObjectInfo.bbx;
          bbx.header = cloudHeader_;
          bbx_pub.publish(bbx);

          // 几何中心
          visualization_msgs::Marker marker;
          VectorXf CAS = VectorXf(5);
          CAS << 0, 1, 0, 0.8, 0.05;
          utility.sphereMarker(cloudHeader_, CAS, detectedObjectInfo.center, marker);
          bbxCenterMarker_pub.publish(marker);                                      
        }       


        // ============================================== 初步定位结果预处理
        //! 发布融合定位结果
        pose_pub.publish(localizationResult.pose);         


        localizedTime = ros::Time::now().toSec(); // 获取到定位结果的时间点，用于将点云定位和KF定位结果进行时间对准
        //------ 转换到locked_pose坐标系
        utility.toLockedPoseFrame(localizationResult.pose, initialCalibrationInfo.p0, localizationResult.lockedFramePose);
        // lockedFramePose_pub.publish(localizationResult.nedPose);
        // 发布locked_pose坐标系下轨迹
        if(debug_publish_){
          std_msgs::ColorRGBA colorRGBA;
          colorRGBA.a = 1.0;
          colorRGBA.g = 1.0;
          string frameID = "locked_pose";
          utility.publishPath(frameID, 
                              colorRGBA, 
                              localizationResult.lockedFramePose.pose.position, 
                              lockedFramePath_pub, 
                              localizationResult.lockedFramePath); 
        }

        if(debug_kf_){
          cout<<"\nDetected object pose"<<endl
              <<"in cloud frame ="<<endl<<localizationResult.pose.pose.position
              <<"in locked_pose frame ="<<endl<<localizationResult.lockedFramePose.pose.position;
        }

        //! 保存到本地文件
        if(savePose_){
          Vector4d lockedFramePose;
          lockedFramePose[0] = localizationResult.lockedFramePose.pose.position.x;
          lockedFramePose[1] = localizationResult.lockedFramePose.pose.position.y;
          lockedFramePose[2] = localizationResult.lockedFramePose.pose.position.z;
          lockedFramePose[3] = localizedTime;
          bbxLockedEst.push_back(lockedFramePose);
        } 


        //------ 转换到NED坐标系
        // if(useNED_){
        //   utility.toNED(localizationResult.pose, initialCalibrationInfo.p0, localizationResult.nedPose);
        //   pose_ned_pub.publish(localizationResult.nedPose);
        // }


            // <<"in NED frame = "<<endl<<localizationResult.nedPose.pose.position<<endl;  

        // //! 发布NED轨迹
        // if(debug_publish_){
        //   std_msgs::ColorRGBA colorRGBA;
        //   colorRGBA.a = 1.0;
        //   colorRGBA.r = 1.0;
        //   string frameID = "ned";
        //   utility.publishPath(frameID, colorRGBA, localizationResult.nedPose.pose.position, nedPath_pub, localizationResult.nedFramePath); 
        // }

        //! 保存到本地文件
        if(savePose_){
          Vector3d ned3f;
          ned3f[0] = localizationResult.nedPose.pose.position.x;
          ned3f[1] = localizationResult.nedPose.pose.position.y;
          ned3f[2] = localizationResult.nedPose.pose.position.z;
          nedEst.push_back(ned3f);
        } 
      

        // ============================================== others 
        // cloudDone = true;
        // detectedObjectInfo.clear();
        
        ROS_INFO_STREAM("Time used since cloud subscribed = "<<ros::Time::now().toSec()-cloudHeader_.stamp.toSec());
        ROS_INFO_STREAM("Time used of cloud processing = "<<ros::Time::now().toSec()-beginTime.toSec());
        std::cout<<"Time of cloud subscribed = "<<cloudHeader_.stamp.toSec()<<std::endl;
        std::cout<<"Time of cloud processing finished = "<<ros::Time::now().toSec()<<std::endl;
        
        newCloudFlag = false;
        cloudMutex.unlock();
        std::cout<<"===================== / cloud processing =========================" << std::endl;
      }
      else
      {
        if(!newCloudFlag){
          ROS_INFO_THROTTLE(5, "Waiting for point cloud ......");
        }else if(!cascadeDetectSuccess){
          ROS_INFO_THROTTLE(5, "Waiting for 2D detector output ......");
        }
        // return;
      }



      // ============================================== Kalman Filter
      // 卡尔曼滤波帮助解决以下问题：（1）2D图像检测器未检测到目标，（2）3D点云聚类未检测到目标（或未订阅到点云话题）
      // 卡尔曼滤波器将使用预测值进行插值补充到目标轨迹中
      if(initialCalibrationInfo.initialized){
          cout<<endl;
          cout<<"Kalman Filter fusion ......"<<endl;
          ros::Time kfBeginTime = ros::Time::now();
                  
          // 时间间隔△t
          float dt = ros::Time::now().toSec() - previousTime_;
          if(debug_kf_) cout<<"dt = "<<dt<<endl;
          previousTime_ = ros::Time::now().toSec();

          // Modify the F matrix so that the time is integrated
          kf.F_(0, 3) = dt;
          kf.F_(1, 4) = dt;
          kf.F_(2, 5) = dt;

          // set the process covariance matrix Q
          float dt2 = dt*dt;
          float dt3 = dt2*dt;
          float dt4 = dt3*dt;
          // kf.Q_ = MatrixXd(6, 6);
          kf.Q_ << dt4/4*kalmanFilaterPara.axNoise, 0, 0, dt3/2*kalmanFilaterPara.axNoise, 0, 0,
                    0, dt4/4*kalmanFilaterPara.ayNoise, 0, 0, dt3/2*kalmanFilaterPara.ayNoise, 0,
                    0, 0, dt4/4*kalmanFilaterPara.azNoise, 0, 0, dt3/2*kalmanFilaterPara.azNoise,
                    dt3/2*kalmanFilaterPara.axNoise, 0, 0, dt2*kalmanFilaterPara.axNoise, 0, 0,
                    0, dt3/2*kalmanFilaterPara.ayNoise, 0, 0, dt2*kalmanFilaterPara.ayNoise, 0,                
                    0, 0, dt3/2*kalmanFilaterPara.azNoise, 0, 0, dt2*kalmanFilaterPara.azNoise;          
          /*kf.Q_ << 1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1, 0,                
                    0, 0, 0, 0, 0, 1;   */         

          // set the measurement covariance matrix R
          // kf.R_ = MatrixXd(3,3);
          kf.R_ << kalmanFilaterPara.xNoise, 0, 0,
                    0, kalmanFilaterPara.yNoise, 0,
                    0, 0, kalmanFilaterPara.zNoise;        

          // predict
          if(debug_kf_) cout<<"Prediction ..."<<endl;
          kf.Predict();
          if(debug_kf_) cout<<"predicted x_ = "<<endl<<kf.x_<<endl;

          // measurement update，融合locked_pose坐标系下的位置
          VectorXd measurements = VectorXd(3);
          measurements[0] = localizationResult.lockedFramePose.pose.position.x;
          measurements[1] = localizationResult.lockedFramePose.pose.position.y;
          measurements[2] = localizationResult.lockedFramePose.pose.position.z;
          if(debug_kf_) cout<<"Update ..."<<endl;
          kf.Update(measurements);        
          if(debug_kf_){
            cout << "fused x_= " << endl<<kf.x_ << endl;
            cout << "P_= " << endl<<kf.P_ << endl;  
          } 

          // 统计时间
          if(debug_kf_) cout<<"Time used of KF = "<<ros::Time::now() - kfBeginTime<<endl;


          // 发布融合定位结果
          localizationResult.fusedPose.header = cloudHeader_;
          localizationResult.fusedPose.header.frame_id = "locked_pose";
          localizationResult.fusedPose.pose.position.x = kf.x_[0];
          localizationResult.fusedPose.pose.position.y = kf.x_[1];
          localizationResult.fusedPose.pose.position.z = kf.x_[2];
          fusedpose_pub.publish(localizationResult.fusedPose);

          // 发布融合目标几何中心marker
          visualization_msgs::Marker fusedMarker;
          VectorXf CAS = VectorXf(5);
          CAS << 0, 0, 1, 0.8, 0.05;
          std_msgs::Header fusedHeader;
          fusedHeader = cloudHeader_;
          fusedHeader.frame_id = "locked_pose";
          utility.sphereMarker(fusedHeader, CAS, kf.x_, fusedMarker);     
          fusedMarker_pub.publish(fusedMarker);

          // 发布融合轨迹
          std_msgs::ColorRGBA fusedPathColor;
          fusedPathColor.b = 1.0;
          fusedPathColor.a = 1.0;
          utility.publishPath(localizationResult.fusedPose.header.frame_id,
                              fusedPathColor,
                              localizationResult.fusedPose.pose.position,
                              fusedPath_pub,
                              localizationResult.fusedPath);

          // 将融合后定位数据写入本地TXT文件
          Vector4d kfTemp;
          kfTemp<<kf.x_[0], kf.x_[1], kf.x_[2], localizedTime;
          kfUpdate.push_back(kfTemp);

      }

      // //! 转换到NED坐标系
      // utility.toNED(localizationResult.pose, initialCalibrationInfo.p0, localizationResult.nedPose);
      // // fusedpose_ned_pub.publish(localizationResult.fusedPoseNED);
      // pose_ned_pub.publish(localizationResult.fusedPoseNED);
      // cout<<"\nFused pose"<<endl
      //     <<"in cloud frame ="<<endl<<localizationResult.fusedPose.pose.position
      //     <<"in NED frame = "<<endl<<localizationResult.fusedPoseNED.pose.position<<endl;

      // //! 保存到本地文件
      // Vector3d ned3f;
      // ned3f[0] = localizationResult.nedPose.pose.position.x;
      // ned3f[1] = localizationResult.nedPose.pose.position.y;
      // ned3f[2] = localizationResult.nedPose.pose.position.z;
      // nedEst.push_back(ned3f);                

      cascadeDetectSuccess  = false;    
    }



    int CloudDetection::updateBBXThread(double clusterTolerance, 
                                        double clusterMinSize, 
                                        double clusterMaxSize)          
    {
        ros::Time bbxTime0 = ros::Time::now();  

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(clusterMinSize);
        ec.setMaxClusterSize(clusterMaxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloudFiltered);
        ec.extract(clusterIndices);

        // for(size_t i=0; i<clusterIndices.size(); i++)
        if(clusterIndices.size() > 0) // 保护机制，解决当未检测到任何聚类情况下，导致节点挂掉问题！
        {
          int i = 0;
          {
              float minX = std::numeric_limits<float>::max();
              float maxX = -std::numeric_limits<float>::max();
              float minY = std::numeric_limits<float>::max();
              float maxY = -std::numeric_limits<float>::max();
              float minZ = std::numeric_limits<float>::max();
              float maxZ = -std::numeric_limits<float>::max();        

              // 循环检查点云聚类中的每一点，找到在3个轴方向的最大最小值
              for(auto pit = clusterIndices[i].indices.begin(); pit != clusterIndices[i].indices.end(); ++pit)
              {
                pcl::PointXYZ p;
                p.x = cloudFiltered->points[*pit].x;
                p.y = cloudFiltered->points[*pit].y;
                p.z = cloudFiltered->points[*pit].z;

                if(p.x < minX) minX = p.x; 
                if(p.y < minY) minY = p.y; 
                if(p.z < minZ) minZ = p.z; 

                if(p.x > maxX) maxX = p.x; 
                if(p.y > maxY) maxY = p.y; 
                if(p.z > maxZ) maxZ = p.z;       
              }


              // 点云聚类在3个轴方向的最大最小值，用于确定3D BBX尺寸
              detectedObjectInfo.bbx.header = cloudHeader_;
              detectedObjectInfo.minPoint.x = minX;
              detectedObjectInfo.minPoint.y = minY;
              detectedObjectInfo.minPoint.z = minZ;
              detectedObjectInfo.maxPoint.x = maxX;
              detectedObjectInfo.maxPoint.y = maxY;
              detectedObjectInfo.maxPoint.z = maxZ;    


              // 3D Bounding Box
              /*
                 _________
                /________/|
                |       | |Z height
                |       | |
                |_______|/ X length
                 Y width

               */
              double length = detectedObjectInfo.maxPoint.x - detectedObjectInfo.minPoint.x;
              double width = detectedObjectInfo.maxPoint.y - detectedObjectInfo.minPoint.y;
              double height = detectedObjectInfo.maxPoint.z - detectedObjectInfo.minPoint.z;
              // BBX filter，将异常大或异常小的BBX恢复到与目标3D mode近似一致大小
              if( (length>objectModel.length*1.2) || (length<objectModel.length*0.8) ) length = objectModel.length;
              if( (width>objectModel.width*1.2) || (width<objectModel.width*0.8) ) width = objectModel.width;
              if( (height>objectModel.height*1.2) || (height<objectModel.height*0.8) ) height = objectModel.height;
              // BBX size
              detectedObjectInfo.bbx.dimensions.x = ((length<0) ? -1*length : length);
              detectedObjectInfo.bbx.dimensions.y = ((width<0) ? -1*width : width);
              detectedObjectInfo.bbx.dimensions.z = ((height<0) ? -1*height : height);            
              // BBX position
              detectedObjectInfo.bbx.pose.position.x = detectedObjectInfo.minPoint.x + length/2;
              detectedObjectInfo.bbx.pose.position.y = detectedObjectInfo.minPoint.y + width/2;
              detectedObjectInfo.bbx.pose.position.z = detectedObjectInfo.minPoint.z + height/2;
              // BBX shape center几何中心（并非质心！）
              detectedObjectInfo.center <<
                                      detectedObjectInfo.minPoint.x + length/2,
                                      detectedObjectInfo.minPoint.y + width/2,
                                      detectedObjectInfo.minPoint.z + height/2;     
          }
        }        


        cout<<"Time used for cloud object detection = " << ros::Time::now() - bbxTime0<<endl; 
        if(clusterIndices.size() == 0){
          ROS_WARN("No cluster is detected!");
        }else{
          cout<<"Detected object number = " << clusterIndices.size(); 
        }

        return clusterIndices.size();
    }


    void CloudDetection::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudIn)
    {
        cloudMutex.lock();
        cloudSubTime = cloudIn->header.stamp;
        cloudHeader_ = cloudIn->header;
        // cout<<"Time when cloud subscribed = "<<cloudSubTime<<endl;


        // ****** 对输入的原始点云进行格式转换，并保存到全局点云变量中。
        // sensor_msgs::PointCloud2类型转换为pcl::PointCloud<pcl::PointXYZ>类型 
        pcl::fromROSMsg(*cloudIn, *cloud);      
        pcl::fromROSMsg(*cloudIn, *cloudRGB);      
        // ROS_INFO_STREAM("initial cloud information    points size: " << cloud->points.size() << "  frame id: " << cloud->header.frame_id);   
        cloudMutex.unlock();    


        if(0 == cloud->points.size())
        {
          ROS_ERROR("Input cloud size is 0 !");
          cloudFilterSucceed = false;
          newCloudFlag = false;
          return;
        }

        cloudFilterSucceed = true;        
        newCloudFlag = true;
    }


    void CloudDetection::cascadeDetectionResultCallback(const jsk_recognition_msgs::RectConstPtr& rect){

        ROS_INFO("Get a 2d detected object rect.");

        cascadeDetectSuccess = true;

        //! 将目标2d bbx的4个顶点转换到相机坐标系下--------------------------------------

        Eigen::Vector2f v1Pixel, v2Pixel, v3Pixel, v4Pixel; // 图像坐标系下的4个顶点，顺时针，左上角到左下角依次为1/2/3/4
        v1Pixel(0) = rect->x;
        v1Pixel(1) = rect->y;
        v2Pixel(0) = rect->x + rect->width;
        v2Pixel(1) = rect->y;    
        v3Pixel(0) = rect->x + rect->width;
        v3Pixel(1) = rect->y + rect->height;    
        v4Pixel(0) = rect->x;
        v4Pixel(1) = rect->y + rect->height;     

        Eigen::Matrix3f intrinsicMatrix;
        intrinsicMatrix << cameraIntrinsic.fx, 0, cameraIntrinsic.cx,
                            0, cameraIntrinsic.fy, cameraIntrinsic.cy,
                            0, 0, 1;

        // 锥体顶面4个顶点
        Eigen::Vector3f v1Cam, v2Cam, v3Cam, v4Cam; // 相机坐标系下的4个顶点，顺时针，左上角到左下角依次为1/2/3/4
        v1Cam = utility.project2dTo3d(v1Pixel, intrinsicMatrix, depthMin_);
        v2Cam = utility.project2dTo3d(v2Pixel, intrinsicMatrix, depthMin_);
        v3Cam = utility.project2dTo3d(v3Pixel, intrinsicMatrix, depthMin_);
        v4Cam = utility.project2dTo3d(v4Pixel, intrinsicMatrix, depthMin_);
        // 锥体底面4个顶点
        // 根据bbx检测结果，动态调整锥体的底面到相机坐标系的距离（用于去除后面干扰点云）：底面距离=bbx.x + bbx.length
        if(adaptiveFrustum && initialCalibrationInfo.initialized && (detectedObjectInfo.bbx.dimensions.x!=0.0)){
          depthMax_ = detectedObjectInfo.center[0] + detectedObjectInfo.bbx.dimensions.x;

        }
        Vector3f v1Cam2, v2Cam2, v3Cam2, v4Cam2; // 相机坐标系下的4个顶点，顺时针，左上角到左下角依次为1/2/3/4
        v1Cam2 = utility.project2dTo3d(v1Pixel, intrinsicMatrix, depthMax_);
        v2Cam2 = utility.project2dTo3d(v2Pixel, intrinsicMatrix, depthMax_);
        v3Cam2 = utility.project2dTo3d(v3Pixel, intrinsicMatrix, depthMax_);
        v4Cam2 = utility.project2dTo3d(v4Pixel, intrinsicMatrix, depthMax_); 
        // 保存梯形锥体的8个顶点，凸包滤波要用到
        detectedObject2D.trapezoid3DVertex.clear();
        detectedObject2D.trapezoid3DVertex.push_back(v1Cam);
        detectedObject2D.trapezoid3DVertex.push_back(v2Cam);
        detectedObject2D.trapezoid3DVertex.push_back(v3Cam);
        detectedObject2D.trapezoid3DVertex.push_back(v4Cam);
        detectedObject2D.trapezoid3DVertex.push_back(v1Cam2);
        detectedObject2D.trapezoid3DVertex.push_back(v2Cam2);
        detectedObject2D.trapezoid3DVertex.push_back(v3Cam2);
        detectedObject2D.trapezoid3DVertex.push_back(v4Cam2);


        //! 绘制梯形锥体--------------------------------------

        //! 梯形椎体，共有8个顶点，12条边，通过绘制12条线marker方式实现在rviz中的显示
        //! 顶点ID：从顶面开始到底面、面四边从左上角开始顺时针。1//2/3/4/5/6/7/8
        std::vector<geometry_msgs::Point> trapezoid3DLine;
        std::vector< std::vector<geometry_msgs::Point> > trapezoid3DAllLine;
        geometry_msgs::Point v1, v2, v3, v4, v5, v6, v7, v8; // ZED相机的点云坐标系为前左上坐标系

        //! 顶面4条边
        v1.x = depthMin_;
        v1.y = -v1Cam(0);
        v1.z = -v1Cam(1);    
        v2.x = depthMin_;
        v2.y = -v2Cam(0);
        v2.z = -v2Cam(1);              
        trapezoid3DLine.push_back(v1);
        trapezoid3DLine.push_back(v2);
        trapezoid3DAllLine.push_back(trapezoid3DLine);         
          
        v3.x = depthMin_;
        v3.y = -v3Cam(0);
        v3.z = -v3Cam(1);            
        trapezoid3DLine.push_back(v2);
        trapezoid3DLine.push_back(v3);
        trapezoid3DAllLine.push_back(trapezoid3DLine); 

        v4.x = depthMin_;
        v4.y = -v4Cam(0);
        v4.z = -v4Cam(1);              
        trapezoid3DLine.push_back(v3);
        trapezoid3DLine.push_back(v4);
        trapezoid3DAllLine.push_back(trapezoid3DLine);      

        trapezoid3DLine.push_back(v4);
        trapezoid3DLine.push_back(v1);
        trapezoid3DAllLine.push_back(trapezoid3DLine);                
        //! 底面4条边
        v5.x = depthMax_;
        v5.y = -v1Cam2(0);
        v5.z = -v1Cam2(1);    
        v6.x = depthMax_;
        v6.y = -v2Cam2(0);
        v6.z = -v2Cam2(1);              
        trapezoid3DLine.push_back(v5);
        trapezoid3DLine.push_back(v6);
        trapezoid3DAllLine.push_back(trapezoid3DLine);         
          
        v7.x = depthMax_;
        v7.y = -v3Cam2(0);
        v7.z = -v3Cam2(1);            
        trapezoid3DLine.push_back(v6);
        trapezoid3DLine.push_back(v7);
        trapezoid3DAllLine.push_back(trapezoid3DLine); 

        v8.x = depthMax_;
        v8.y = -v4Cam2(0);
        v8.z = -v4Cam2(1);              
        trapezoid3DLine.push_back(v7);
        trapezoid3DLine.push_back(v8);
        trapezoid3DAllLine.push_back(trapezoid3DLine);      

        trapezoid3DLine.push_back(v8);
        trapezoid3DLine.push_back(v5);
        trapezoid3DAllLine.push_back(trapezoid3DLine);   
        //! 腰上4条边             
        trapezoid3DLine.push_back(v1);
        trapezoid3DLine.push_back(v5);
        trapezoid3DAllLine.push_back(trapezoid3DLine);                     
        trapezoid3DLine.push_back(v2);
        trapezoid3DLine.push_back(v6);
        trapezoid3DAllLine.push_back(trapezoid3DLine);              
        trapezoid3DLine.push_back(v3);
        trapezoid3DLine.push_back(v7);
        trapezoid3DAllLine.push_back(trapezoid3DLine);      
        trapezoid3DLine.push_back(v4);
        trapezoid3DLine.push_back(v8);
        trapezoid3DAllLine.push_back(trapezoid3DLine);           

        std_msgs::ColorRGBA color;
        color.b = 1.0;
        color.a = 1.0;
        double width(0.005);                
        utility.publish3DConvexHullMarkers("zed_left_camera_frame", color, width, trapezoid3DAllLine, trapezoid3D_pub);
        //-----------------------------------------------------------------

    }


    bool CloudDetection::modelCheck()
    {
        // 尺寸大小检查
        bool modelSize = (detectedObjectInfo.bbx.dimensions.x > objectModel.length * objectModel.sFactor_min)&& // >=
                        (detectedObjectInfo.bbx.dimensions.y > objectModel.width * objectModel.sFactor_min)&&
                        (detectedObjectInfo.bbx.dimensions.z > objectModel.height * objectModel.sFactor_min)&&
                        (detectedObjectInfo.bbx.dimensions.x < objectModel.length * objectModel.sFactor_max)&& // <=
                        (detectedObjectInfo.bbx.dimensions.y < objectModel.width * objectModel.sFactor_max)&&
                        (detectedObjectInfo.bbx.dimensions.z < objectModel.height * objectModel.sFactor_max)&&                        
                        (cloudFiltered->size() > objectModel.nFactor * objectModel.cloudSize);
        if(!modelSize){
          ROS_WARN("Model size check not passed.");
          return false;
        }

        // 形状相似性检查
        float l12 = detectedObjectInfo.bbx.dimensions.x / objectModel.length;
        float w12 = detectedObjectInfo.bbx.dimensions.y / objectModel.width;
        float h12 = detectedObjectInfo.bbx.dimensions.z / objectModel.height;
        bool modelProportion = (l12/w12 > objectModel.pFactor) && (l12/h12 > objectModel.pFactor) && (w12/h12 > objectModel.pFactor);
        if(!modelProportion){
          ROS_WARN("Model proportionality check not passed.");
          return false;
        }

        return true;      
    }



    void CloudDetection::testThreadFun(int a, int& b, Matrix4f& c, PointCloud& cloudFiltered)
    {
      a = 11;
      b = 22;
      c.setIdentity();
      c(1,2) = 55;
      // sleep(5);
      while(ros::ok()){
        ROS_INFO_THROTTLE(1, "thread 1......");
        ROS_INFO_STREAM_THROTTLE(1, "point cloud size="<<cloudFiltered.size());
        // cout<<"point cloud size="<<cloudFiltered.size()<<endl;
        // cout<<"thread 2......"<<endl;
      }      
    }

    void CloudDetection::testThreadFun2(int aa, int& bb, Matrix4f& cc)
    {
      aa = 11;
      bb = 22;
      cc.setIdentity();
      cc(1,2) = 55;
      // sleep(2);
      while(ros::ok()){
        ROS_INFO_THROTTLE(1, "thread 2......");
        // cout<<"thread 2......"<<endl;
      }
    }

 
    void CloudDetection::matrix2angleANDtransition(
      Eigen::Matrix4f &T, 
      Eigen::Vector3f &angle,
      Eigen::Vector3f &transition){
        // get angle
        double ax, ay, az;
        if(T(2,0)==1 || T(2,0)==-1){
            az = 0;
            double delta;
            delta = atan2(T(0,1), T(0,2));
            if(T(2,0)==-1){
                ay = M_PI/2;
                ax = az + delta;
            }else{
                ay = M_PI/2;
                ax = -az + delta;
            }
        }else{
            ay = -asin(T(2,0));
            ax = atan2(T(2,1)/cos(ay), T(2,2)/cos(ay));
            az = atan2(T(1,0)/cos(ay), T(0,0)/cos(ay));
        }
        angle<<ax,ay,az;

      // get transition
      transition<<T(0,3),T(1,3),T(2,3);
    }



    bool CloudDetection::saveCloud(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
      ROS_WARN("save cloud to FILE ......");

      cloudMutex.lock();
      // pcl::io::savePCDFileASCII("initial_cloud.pcd", *cloud);

      pcl::io::savePCDFile(saveCloudDir_+"initial.pcd", *cloud);
      ROS_INFO_STREAM("Saved initial cloud to initial_cloud.pcd, cloud size="<<cloud->points.size());
      
      if(newCloudFlag && cascadeDetectSuccess){
        pcl::io::savePCDFile(saveCloudDir_+"filtered.pcd", *cloudFiltered);
        ROS_INFO_STREAM("Saved filtered cloud to filtered_cloud.pcd, cloud size="<<cloudFiltered->points.size());
      }
      
      pcl::io::savePCDFile(saveCloudDir_+"initial_RGB.pcd", *cloudRGB);

      cloudMutex.unlock();

      return true;
    }

    // ****** 保存数据到本地txt文件
    bool CloudDetection::saveLog(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
      ROS_WARN("save data to TXT ......");

      std::string username = utility.getUserName();
      // =========================== estimates 点云定位原始结果(点云坐标系)
      // std::string filePath = "/home/chenlu/est.txt";
      std::string filePath = "/home/"+username+"/data/log/est_cloudframe.txt";      
      std::ofstream outFile1;    
      outFile1.open(filePath, std::ios::app|std::ios::out);
      outFile1<<std::fixed;
      outFile1.precision(3);
      if(outFile1.bad()){
          cout<<"cannot create file: "<<filePath<<endl;
          return 1;
      }
      for(int i=0; i<bluerov_detection_tracking::bbxEst.size(); i++){
          outFile1<<bluerov_detection_tracking::bbxEst[i][0]<<"\t"<<bluerov_detection_tracking::bbxEst[i][1]<<"\t"<<bluerov_detection_tracking::bbxEst[i][2]<<endl;
      }
      outFile1.close();
      // std::cout<<"estimates saved in file = "<<filePath<<endl;  
      ROS_WARN_STREAM("bbx pose estimates saved in file = "<<filePath);


      // =========================== estimates 点云定位原始结果(locked_pose坐标系)
      filePath = "/home/"+username+"/data/log/est_lockedframe.txt";      
      std::ofstream lockedEstFile;    
      lockedEstFile.open(filePath, std::ios::app|std::ios::out);
      lockedEstFile<<std::fixed;
      lockedEstFile.precision(3);
      if(lockedEstFile.bad()){
          cout<<"cannot create file: "<<filePath<<endl;
          return 1;
      }
      for(int i=0; i<bluerov_detection_tracking::bbxLockedEst.size(); i++){
          lockedEstFile<<bluerov_detection_tracking::bbxLockedEst[i][0]<<"\t"<<bluerov_detection_tracking::bbxLockedEst[i][1] \
          <<"\t"<<bluerov_detection_tracking::bbxLockedEst[i][2]<<"\t"<<bluerov_detection_tracking::bbxLockedEst[i][3]<<endl;
      }
      lockedEstFile.close();
      ROS_WARN_STREAM("bbx pose(in locked_pose frame) estimates saved in file = "<<filePath);


      // =========================== KF states. KF融合平滑滤波结果(locked_pose坐标系)
      if(useKF_){
        filePath = "/home/"+username+"/data/log/kf.txt";
        ofstream kfFile;
        kfFile.open(filePath, std::ios::app|std::ios::out);
        kfFile<<std::fixed;
        kfFile.precision(3);
        if(kfFile.bad()){
          cout<<"cannot open file = "<<filePath<<endl;
          return 1;
        }
        for(unsigned int i=0; i<kfUpdate.size(); ++i){
          kfFile<<kfUpdate[i][0]<<"\t"<<kfUpdate[i][1]<<"\t"<<kfUpdate[i][2]<<"\t"<<kfUpdate[i][3]<<endl;
        }
        kfFile.close();
        ROS_WARN_STREAM("fused estimates saved in file = "<<filePath);
      }


      // =========================== NED pose estimates
      filePath = "/home/chenlu/log/ned.txt";
      std::ofstream outFile2;    
      outFile2.open(filePath, std::ios::app|std::ios::out);
      outFile2<<std::fixed;
      outFile2.precision(3);
      if(outFile2.bad()){
          cout<<"cannot create file: "<<filePath<<endl;
          return 1;
      }
      for(int i=0; i<bluerov_detection_tracking::nedEst.size(); i++){
          outFile2<<bluerov_detection_tracking::nedEst[i][0]<<"\t"<<bluerov_detection_tracking::nedEst[i][1]<<"\t"<<bluerov_detection_tracking::nedEst[i][2]<<endl;
      }
      outFile2.close();
      ROS_WARN_STREAM("NED pose estimates saved in file = "<<filePath);


      // =========================== ground truth
      if(hasGT_){
        filePath = "/home/"+username+"/data/log/gt.txt";
        ofstream outFile2;
        outFile2.open(filePath, std::ios::app|std::ios::out);
        outFile2<<std::fixed;
        outFile2.precision(3);
        if(outFile2.bad()){
            cout<<"cannot create file: "<<filePath<<endl;
            return 1;
        }    
        for(int i=0; i<bluerov_detection_tracking::gt.size(); i++){
            outFile2<<bluerov_detection_tracking::gt[i][0]<<"\t"<<bluerov_detection_tracking::gt[i][1]<<"\t"<<bluerov_detection_tracking::gt[i][2]<<endl;
        }
        outFile2.close();
        ROS_WARN_STREAM("groud truth pose saved in file = "<<filePath);
      }               

      return true; 
    }

    bool CloudDetection::computeRMSE(pcl::ComputeRMSE::Request &request, pcl::ComputeRMSE::Response &response)
    {
      // ****** 计算RMSE
      vector<Vector3d> est, gt;
      std::string filePath;

      // load estimation data.
      // filePath = "/home/chenlu/estimates.txt";
      filePath = "/home/chenlu/" + request.file1;
      std::ifstream inFile1;
      inFile1.open(filePath, ios::in);
      if(!inFile1){
          cout<<"cannot open file: "<<filePath<<endl;
          return 1;
      }
      while(!inFile1.eof()){
          Vector3d temp;
          inFile1>>temp[0]>>temp[1]>>temp[2];
          est.push_back(temp);
      }
      inFile1.close();

      // load ground truth data.
      // filePath = "/home/chenlu/groundtruth.txt";
      filePath = "/home/chenlu/" + request.file2;
      ifstream inFile2;
      inFile2.open(filePath, ios::in);
      if(!inFile2){
          cout<<"cannot open file: "<<filePath<<endl;
          return 1;
      }
      while(!inFile2.eof()){
          Vector3d temp;
          inFile2>>temp[0]>>temp[1]>>temp[2];
          gt.push_back(temp);
      }
      inFile2.close();    

      // RMSE
      Vector3d rmse;
      rmse = calculateRMSE(est, gt);
      ROS_WARN("compute RMSE ......");
      cout<<"RMSE = "<<endl<<rmse<<endl;   

      response.rmse.x = rmse[0];
      response.rmse.y = rmse[1];
      response.rmse.z = rmse[2];

      return true;  
    }


    Vector3d CloudDetection::calculateRMSE(vector<Vector3d> &est, vector<Vector3d> &gt)
    {
      cout<<"calculate RMSE."<<endl;
      Vector3d rmse;
      rmse<<0,0,0;

      // check input data valid.
      if(est.size()==0){
        cout<<"input data error!"<<endl;
        return rmse;
      }

      // RMSE
      for(unsigned int i=0; i<est.size(); ++i){
        Vector3d residual = est[i] - gt[i];
        residual = residual.array() * residual.array();
        rmse += residual;
      }
      rmse = rmse / est.size();
      rmse = rmse.array().sqrt();

      return rmse;
    }
        

} // namespace

/*
/republish_cxx_node/KF_marker/pose/position/x
/republish_cxx_node/bounding_box_center/markers[0]/pose/position/x
*/

