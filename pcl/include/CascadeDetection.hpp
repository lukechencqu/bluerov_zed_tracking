#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/Rect.h>

// #include <sensor_msgs/Point_cloud2.h>
#include <jsk_recognition_msgs/BoundingBox.h>

// opencv.
// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/Core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
// pcl.
// #include <pcl/pcl.h>

//! Eigen
#include <Eigen/Dense>

//! user
#include "Utility.hpp"
#include "KalmanFilter.h"

using namespace std;
using namespace Eigen;

//http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
//http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage

namespace bluerov_detection_tracking
{

    KalmanFilter kf;


    struct KalmanFilterPara
    {
        // Q，加速度噪声
        float axNoise;
        float ayNoise;

        // R, 传感器噪声  
        float xNoise;
        float yNoise;
    } kalmanFilaterPara;


class CascadeDetection
{

public:
    // Registration(ros::NodeHandle& node);
    // virtual ~Registration();
    CascadeDetection(ros::NodeHandle& node);
    virtual ~CascadeDetection();


    bool readParameters();


    bool initilization();


    /*
        RGB图像及相机内参订阅回调函数
        输入：ros RGB图像话题
             ros 相机参数话题
        返回：发布处理后的图像（包含目标bbx方框）
             发布目标rect话题
    */
    void imgCamCallback(const sensor_msgs::ImageConstPtr& imgMsg, 
                        const sensor_msgs::CameraInfoConstPtr& infoMsg);


    /*
        RGB图像回调函数
        输入：ros RGB图像话题
        返回：发布处理后的图像（包含目标bbx方框）
             发布目标rect话题
    */
    void imgCallback(const sensor_msgs::ImageConstPtr& imgMsg);



    /*
        订阅由3D点云检测到的BBX的回调函数
        输入：
        返回：
    */
    void bbx3dCallback(const jsk_recognition_msgs::BoundingBoxConstPtr& msg);


    cv::Rect detect(cv::Mat& img, 
                    cv::CascadeClassifier& cascade,
                    cv::CascadeClassifier& nestedCascade,
                    double scale );    



private:

    ros::NodeHandle node_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber imgSub_; // 只订阅图像
    image_transport::CameraSubscriber imgCamSub_; // 同时订阅图像及相机参数
    image_transport::Publisher imgPub_; // 发布处理后的图像

    ros::Publisher reprojected3DBBXcenterMarkerPub_; // 目标2d bbx中心投影到相机坐标系下的3d bbx中心
    ros::Publisher frustum_pub; // 锥体（由ROV 2D bbx投影生成，顶点位于相机坐标系原点），用于RVIZ显示
    ros::Publisher trapezoid3D_pub; // 梯形锥体（由ROV 2D bbx投影生成），用于RVIZ显示  
    ros::Publisher rovRect_pub; // 最终检测到的ROV rect框

    ros::Subscriber cloudSub_;
    ros::Subscriber poseSub_;
    ros::Subscriber bbx3dSub_;

    image_geometry::PinholeCameraModel camModel_;

    Utility utility_;

    // 3D bbx投影到图像平面后生成的2D bbx几何参数
    cv::Point2d center_; // 目标box中心
    cv::Point2d pp1PixelFrame_, pp2PixelFrame_, pp3PixelFrame_, pp4PixelFrame_;
    cv::Point2i roiP1_, roiP2_;

    geometry_msgs::Point center3d_;


    //! 相机内参----------------------------------------------------------
    // # Intrinsic camera matrix for the raw (distorted) images.
    // #     [fx  0 cx]
    // # K = [ 0 fy cy]
    // #     [ 0  0  1]
    // # Projects 3D points in the camera coordinate frame to 2D pixel
    // # coordinates using the focal lengths (fx, fy) and principal point
    // # (cx, cy).    
    Matrix3f K_;  

    /*
    # Projection/camera matrix
    #     [fx'  0  cx' Tx]
    # P = [ 0  fy' cy' Ty]
    #     [ 0   0   1   0]
    # By convention, this matrix specifies the intrinsic (camera) matrix
    #  of the processed (rectified) image. That is, the left 3x3 portion
    #  is the normal camera intrinsic matrix for the rectified image.
    # It projects 3D points in the camera coordinate frame to 2D pixel
    #  coordinates using the focal lengths (fx', fy') and principal point
    #  (cx', cy') - these may differ from the values in K.
    # For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
    #  also have R = the identity and P[1:3,1:3] = K.
    # For a stereo pair, the fourth column [Tx Ty 0]' is related to the
    #  position of the optical center of the second camera in the first
    #  camera's frame. We assume Tz = 0 so both cameras are in the same
    #  stereo image plane. The first camera always has Tx = Ty = 0. For
    #  the right (second) camera of a horizontal stereo pair, Ty = 0 and
    #  Tx = -fx' * B, where B is the baseline between the cameras.
    # Given a 3D point [X Y Z]', the projection (x, y) of the point onto
    #  the rectified image is given by:
    #  [u v w]' = P * [X Y Z 1]'
    #         x = u / w
    #         y = v / w
    #  This holds for both images of a stereo pair.
    float64[12] P # 3x4 row-major matrix
    */
    MatrixXf P_;   
    //--------------------------------------------------------------- 


    //! 输入的原始RGB图像
    cv::Mat image_;
    
    //! 目标模型cascade文件路径名称
    // https://docs.opencv.org/3.4.9/d1/de5/classcv_1_1CascadeClassifier.html
    string cascadeName_;
    string nestedCascadeName_;
    cv::CascadeClassifier cascade_;
    cv::CascadeClassifier nestedCascade_;


    //! detectMultiScale()检测器函数参数 -------------------------
    //! 允许检测到的目标最大和最小尺寸（相对于图像尺寸）
    double minSizeScale_; // 整体目标检测器
    double maxSizeScale_; 
    double maxColSizeScaleNested_, maxRowSizeScaleNested_; // 嵌套局部检测器
    double minColSizeScaleNested_, minRowSizeScaleNested_;    
    //! 要求在同一位置连续检测到目标的数量
    int continuousDetectNum_;
    int continuousDetectNumNested_; 

    //! 其他检测用参数
    double scale_; // 检测图像缩小的倍数（默认1.0），缩小检测图像可以提高检测速度，但会降低检测成功率
    bool tryflip_;
    double rectNestedScale_;
    //--------------------------------------------------------


    bool debug_;

    //! 最大有效检测景深，摄像机坐标系，单位：米
    double filterDepthValue_;

    //! 最终目标确认标志
    bool confirmedObjectFlag_;

    //! 最终目标bbx
    cv::Rect confirmedROV_;

    // 根据相机是否支持输出相机内参信息的能力来决定使用哪个图像话题订阅器
    bool imageAndCameraInfoCallback_;

    string videoName_;
    cv::VideoCapture capture;




};

}
