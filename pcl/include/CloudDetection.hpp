#ifndef CLOUD_DETECTION_H
#define CLOUD_DETECTION_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/Rect.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/intersections.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/features/feature.h> // features
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/boundary.h>
// #include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h> // segment
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

// STL
#include <iostream>
#include <fstream>
#include <thread>

// EIGEN
#include <Eigen/Dense>

// system information
#include <unistd.h>
#include <pwd.h>

// 自定义
#include "PointcloudFilter.hpp"
#include "KalmanFilter.h"
#include "Utility.hpp"
#include "pcl/ComputeRMSE.h"

#define PI 3.1415926

using namespace Eigen;
using namespace std;

// ========================================= struct


namespace bluerov_detection_tracking
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

    // DEBUG
    bool debug_ = false;
    bool debug_ICP_ = true;
    
    // localization results.
    std::vector<Eigen::Vector3d> bbxEst; // 原始点云定位（点云坐标系），
    std::vector<Eigen::Vector4d> bbxLockedEst; // 原始点云定位（locked_pose坐标系），
    std::vector<Eigen::Vector4d> kfUpdate; // KF平滑定位（locked_pose坐标系），    
    std::vector<Eigen::Vector3d> icpEst, nedEst; // 暂停使用
    vector<VectorXf> gt;      



    //! 其他类 -------------------------------------

    // 通用功能函数
    Utility utility;

    // 点云滤波
    PointCloudFilter cloudFilter;

    // kalman filter
    KalmanFilter kf;


    //! 结构体 ------------------------------------
    
    struct KalmanFilterPara
    {
        // Q，加速度噪声
        float axNoise;
        float ayNoise;
        float azNoise; 

        // R, 传感器噪声  
        float xNoise;
        float yNoise;
        float zNoise;       
    } kalmanFilaterPara;

    //! 摄像机内参
    struct CameraIntrinsic
    {
        double fx;
        double fy;
        double cx;
        double cy;
    } cameraIntrinsic;    

    // 保存2D目标检测结果信息
    struct DetectedObject2D
    {
        std::vector<Eigen::Vector3f> trapezoid3DVertex; // 相机坐标系（z轴朝前）
    } detectedObject2D;



    // 被成功检测到的目标的基本信息
    struct DetectedObjectInfo
    {
        jsk_recognition_msgs::BoundingBox bbx; // 目标3D BBX

        pcl::PointXYZ minPoint; // 目标聚类点云的最小位置点
        pcl::PointXYZ maxPoint; // 目标聚类点云的最大位置点
        pcl::PointXYZ centroid;  // 目标聚类点云的重心
        //pcl::PointXYZ center_;
        Eigen::Vector3d center; // 目标 3D BBX几何中心
    } detectedObjectInfo;   


    // 保存最终目标定位结果信息
    struct LocalizationResult
    {
        geometry_msgs::PoseStamped pose; // 融合前的位姿（点云坐标系）
        geometry_msgs::PoseStamped lockedFramePose; // 融合前的位姿（locked_pose坐标系）
        geometry_msgs::PoseStamped nedPose; // 融合前的位姿（NED坐标系）        
        geometry_msgs::PoseStamped fusedPose; // 融合后的位姿（locked_pose坐标系）
        geometry_msgs::PoseStamped fusedPoseNED; // 融合后的位姿（NED坐标系）

        visualization_msgs::Marker lockedFramePath;
        visualization_msgs::Marker nedFramePath;
        visualization_msgs::Marker fusedPath;

    } localizationResult;



    //! ROV初始化标定参数
    struct InitialCalibrationPara
    {
        // 目标位置初始化参数
        float poseFluctuationX, poseFluctuationY, poseFluctuationZ; // 目标在三个轴方向允许波动的范围
        int consecutiveTinyFluctuationTimes; // 连续保持在允许波动范围内的次数，只有大于此阈值才判定位置初始化成功
    } initialCalibrationPara;


    //! ROV初始化标定成功以后保存的信息
    struct InitialCalibrationInfo
    {
        // 梯形锥体深度范围：用于点云ROI滤波
        float trapezoidMinDepth; // 顶面到相机中心距离
        float trapezoidMaxDepth; // 底面到相机中心距离

        // 初始参考点云相关信息：作为先验知识，用于后续目标匹配
        PointCloud::Ptr referenceCloud; // 目标初始点云
        Eigen::Vector3f p0; // 目标点云初始位置，位于cloud frame(if ZED, then X=frontward)：用于构建NED坐标系
        jsk_recognition_msgs::BoundingBox bbx; // 目标初始点云的包围盒，作为目标匹配的先验知识（尺寸、长宽高比例）     

        // 初始化完成的标志
        bool initialized; // 所有初始化过程已经成功完成
        bool poseInitialized; // 位置初始化完成
        bool trapezoidInitialized; // 梯形锥体深度范围初始化完成
        bool cloudInitialized; // 成功获取目标初始参考点云
    } initialCalibrationInfo;


    struct Calibration
    {
        float kx;
        float ky;
        float kz;
    } calibration_;

    struct ObjectModel
    {
        float length;
        float width;
        float height;
        int cloudSize;
        float nFactor;
	    float sFactor_min;
        float sFactor_max;
        float pFactor;
        bool useModelCheck;        
    } objectModel;

 

    //! 类 -------------------------------------
    class CloudDetection
    {

    public:
                    
        // CloudDetection(ros::NodeHandle& node);
        CloudDetection();
        virtual ~CloudDetection();

        bool readParameters(const ros::NodeHandle& node);

        bool initialize(const ros::NodeHandle& n);

        bool registerCallbacks(const ros::NodeHandle& n);



        bool saveCloud(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); 

        bool saveLog(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); 

        bool computeRMSE(pcl::ComputeRMSE::Request &request, pcl::ComputeRMSE::Response &response);

        Vector3d calculateRMSE(vector<Vector3d> &est, vector<Vector3d> &gt);

        void publishROI();

        void publishPath(string frame_id, std_msgs::ColorRGBA colorRGBA);
        

    private:

        /*
            目标位置初始化标定：获取目标对准的初始位置
            输入：上一时刻位置
                 当前位置
            输出：初始化后的平均初始位置
            返回：位置初始化成功标志
        */
        bool poseInitialization(Eigen::Vector3d previousPose, 
                                Eigen::Vector3d currentPose);


        bool modelCheck();

        void toNED();


        void matrix2angleANDtransition(Eigen::Matrix4f &T, Eigen::Vector3f &angle, Eigen::Vector3f &transition);


        /*
            点云聚类检测目标
            输入：聚类参数
            输出：聚类结果点云
            返回：检测到的聚类数量
        */
        int updateBBXThread(double clusterTolerance, 
                            double clusterMinSize, 
                            double clusterMaxSize);            


        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudIn);

        void cloudProcess(const ros::TimerEvent& event);    
            
        void cascadeDetectionResultCallback(const jsk_recognition_msgs::RectConstPtr& rect);            


        //! 以下代码仅供测试用
        void testThreadFun(int a, int& b, Matrix4f& c, PointCloud& cloudFiltered);
        void testThreadFun2(int aa, int& bb, Matrix4f& cc);


    public:
        // 是否订阅到新的点云标志？
        bool newCloudFlag = false;

        // 是否订阅到了2D目标检测结果标志？
        bool cascadeDetectSuccess;


    private:
        // ros::NodeHandle& node_;

        boost::mutex cloudMutex;

        // ========================================= publisher
        // 点云
        ros::Publisher cloud_pub;
        ros::Publisher incloudFiltered_pub;
        ros::Publisher referenceCloud_pub, readingCloud_pub, alignedCloud_pub;
        // 图形
        ros::Publisher referenceBBX_pub;
        ros::Publisher bbx_pub;
        ros::Publisher bbxCenterMarker_pub;
        ros::Publisher fusedMarker_pub;
        ros::Publisher ROI_pub;

        ros::Publisher frustum_pub; // 锥体（由ROV 2D bbx投影生成，顶点位于相机坐标系原点），用于RVIZ显示
        ros::Publisher trapezoid3D_pub; // 梯形锥体（由ROV 2D bbx投影生成），用于RVIZ显示          


        // 发布定位坐标
        ros::Publisher pose_pub;
        ros::Publisher pose_ned_pub;
        ros::Publisher fusedpose_pub;
        ros::Publisher fusedpose_ned_pub;
        // 发布定位轨迹
        ros::Publisher posePath_pub; // 目标位置在点云坐标系下的轨迹
        ros::Publisher lockedFramePath_pub; // 目标位置在locked_pose坐标系下的轨迹
        ros::Publisher nedPath_pub; // 目标位置在NED坐标系下的轨迹
        ros::Publisher fusedPath_pub; // 目标位置融合之后的轨迹



        // ========================================= subscriber
        ros::Subscriber cloud_sub;
        ros::Subscriber odomLeader_sub, odomFollower_sub;

        ros::Subscriber cascadeDetectionResult_sub; // 图像目标检测结果（2D RECT）

        // ========================================= service
        ros::ServiceServer saveCloudService;
        ros::ServiceServer saveLogService;
        ros::ServiceServer computeRMSEService;

        // ========================================= Timer
        ros::Timer cloudTimer_;


        // ========================================= global variables
        // 初始化步骤所用到的参数
        int consecutiveTinyFluctuationCout_; // 目标位置已经连续保持在波动阈值范围内的计数
        Eigen::Vector3d consecutiveTinyFluctuationPoseSum_; // 目标位置已经连续保持在波动阈值范围内的位置和
        Eigen::Vector3d previousPose_; // 上一时刻目标位置 bbxTK1, bbxTK2;
        bool firstBBX_; // 第一次检查到目标标志

        // 目标位置变化量（相对于初始位置）
        Eigen::Vector3d poseDelta_;


        static geometry_msgs::Point odomInitialPose, odomCurrentCalledPose, odomPreviousCalledPose;
        static geometry_msgs::Point pointcloud_initial_pose, cloud_current_pose, cloud_previous_pose;
        geometry_msgs::Pose odomPoseChanged, pointcloudPoseChanged, poseChangedBias, poseChangedBias_current, poseChangedBias_previous;
        geometry_msgs::Pose odomPoseDelta, odomPoseCurrent, odomPosePrevious, odomPoseAccumulate;
        bool cloudFilterSucceed = false;
        int pointcloudPoseChangedMsgID = 0;
        bool cloudInitialized = false; // for the first time that cloud subscribed.

        ros::Time cloudSubTime;

        int previousTime_; // 卡尔曼滤波前一时刻时间


        // ========================================= 从参数服务器加载的可供用户配置的parameters
        bool useThread_;
        bool useKF_;
        bool hasGT_; // ground truth
        bool useDynamicROI_;
        bool savePose_; // 保存定位结果到本地日志文件
        bool useNED_;
        bool worldFrame_; 
        bool useRelativePose_;


        // cluster
        float clusterTolerance_, clusterMinSize_, clusterMaxSize_;

        //! cloud saved file directory.
        std::string saveCloudDir_;

        // ICP
        int icpMethod_;
        int setMaximumIterations_;
        float setTransformationEpsilon_;
        float setMaxCorrespondenceDistance_;
        float setEuclideanFitnessEpsilon_;
        int setRANSACIterations_;    

        // Timer
        float frequency_;

        // debug flag
        bool debug_publish_;
        bool debug_initialize_;
        bool debug_kf_;
        bool debug_time_;

        //! topics
        std::string odomLeaderTopic_, odomFollowerTopic_, cloudTopic_;  
        std::string rectTopic_; // 图像目标检测结果
        


        Vector3d icpPose;

        //! 线程
        std::thread bbxThread_;
        // 点云包头信息
        std_msgs::Header cloudHeader_;

        //! ROI
        float roiL_, roiW_, roiH_;


        //! 2D目标bbx投影到3D锥体 -----------------
        float depthMin_, depthMax_; // 生成的梯形锥体的顶面和底面分别到相机坐标系原点的距离

        // 目标位置跳变处理用到的参数
        Eigen::Vector3d poseJumpBiasThreshold_;
    }; // class

} // namespace



#endif
