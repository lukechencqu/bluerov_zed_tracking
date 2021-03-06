#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/Point_cloud2.h>
#include <jsk_recognition_msgs/BoundingBox.h>

// opencv.
// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/Core/core.hpp>
#include <cv_bridge/cv_bridge.h>

// pcl.
// #include <pcl/pcl.h>

//! Eigen
#include <Eigen/Dense>

//! user
#include "Utility.hpp"

using namespace std;
using namespace Eigen;

//http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
//http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage

class Registration
{
    ros::NodeHandle node_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber imgSub_;
    image_transport::CameraSubscriber imgCamSub_;
    image_transport::Publisher imgPub_;

    ros::Subscriber cloudSub_;
    ros::Subscriber poseSub_;
    ros::Subscriber bbxSub_;

    image_geometry::PinholeCameraModel camModel_;

public:
    // Registration(ros::NodeHandle& node);
    // virtual ~Registration();
    Registration(ros::NodeHandle& node)
        : node_(node),
        it_(node)
    {
        cout<<"imgCloudRegistration"<<endl;

        string imgTopic = node_.resolveName("/zed/zed_node/left/image_rect_color");
        string cloudTopic = node_.resolveName("/zed/zed_node/point_cloud/cloud_registered");
        string poseTopic = node_.resolveName("/republish_cxx_node/bbxpose");
        string bbxTopic = node_.resolveName("/republish_cxx_node/bbx");
        // cout<<"resolved image topic name: "<<imgTopic<<endl;

        imgCamSub_ = it_.subscribeCamera(imgTopic, 10, &Registration::imgCamCallback, this);
        // imgSub_ = it_.subscribe(imgTopic, 10, &Registration::imgCallback, this);

        // http://wiki.ros.org/image_transport/Tutorials/PublishingImages
        imgPub_ = it_.advertise("image", 10);

        cloudSub_ = node_.subscribe(cloudTopic, 10, &Registration::cloudCallback, this);
        // poseSub_ = node_.subscribe(poseTopic, 10, &Registration::poseCallback, this);
        bbxSub_ = node_.subscribe(bbxTopic, 10, &Registration::bbxCallback, this);
    }

    // bool initilization()
    // {
    //     // ros::NodeHandle node2(node);
    //     cout<<"Registration"<<endl;

    //     string imgTopic = node_.resolveName("/zed/zed_node/left/image_rect_color");
    //     string cloudTopic = node_.resolveName("/zed/zed_node/point_cloud/cloud_registered");
    //     cout<<"resolved image topic name: "<<imgTopic<<endl;

    //     it_(node_);
    //     imgSub_ = it_.subscribeCamera(imgTopic, 10, &Registration::imgCallback, this);
    //     cloudSub_ = node_.subscribe(cloudTopic, 10, &Registration::cloudCallback, this);
    //     // imgSub_ = it_.subscribeCamera(imgTopic, 10, &Registration::imgCallback, this);
    //     // cloudSub_ = node_.subscribe("/zed/zed_node/point_cloud/cloud_registered", 10, &Registration::cloudCallback, this);
    // }    

    void imgCamCallback(const sensor_msgs::ImageConstPtr& imgMsg, const sensor_msgs::CameraInfoConstPtr& infoMsg){
        cout<<"image info callback"<<endl;

        //! intrinsic parameters.
        // cout<<infoMsg->K[0]<<endl;
        // cout<<infoMsg->K[1]<<endl;
        // cout<<infoMsg->K[2]<<endl;
        // cout<<infoMsg->K[3]<<endl;
        // cout<<infoMsg->K[4]<<endl;
        // cout<<infoMsg->K[5]<<endl;
        // cout<<infoMsg->K[6]<<endl;
        // cout<<infoMsg->K[7]<<endl;
        // cout<<infoMsg->K[8]<<endl;

        //! 获取相机的内参
        K_ << infoMsg->K[0],0,infoMsg->K[2],
            0,infoMsg->K[4],infoMsg->K[5],
            0,0,1;        
        // K_ << 677.393,0,620.749,
        //     0,677.393,350.597,
        //     0,0,1;    
          
        P_.resize(3,4);  
        P_ << infoMsg->P[0],0,infoMsg->P[2],infoMsg->P[3],
            0,infoMsg->P[5],infoMsg->P[6],infoMsg->P[7],
            0,0,1,0;    

        camModel_.fromCameraInfo(infoMsg);
        // cout<<"camera model: "<<camModel_<<endl;


        //! 获取RGB图像，转成opencv格式，并处理后发布出去
        // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
        cv::Mat img;
        cv_bridge::CvImagePtr br;
        try{
            br = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
            img = br->image;
            // cout<<"input image size: "<<img.cols<<"  "<<img.rows<<endl;
        }catch(cv_bridge::Exception& ex){
            ROS_ERROR("Failed to convert image!");
            return;
        }

        //! draw markers on image. http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage
        cv::circle(img, center_, 10, CV_RGB(255,0,0), -1);
        //! BBX四条边的中点
        cv::circle(img, pp1PixelFrame_, 8, CV_RGB(255,0,0), -1);
        cv::circle(img, pp2PixelFrame_, 8, CV_RGB(0,255,0), -1);
        cv::circle(img, pp3PixelFrame_, 8, CV_RGB(0,0,255), -1);
        cv::circle(img, pp4PixelFrame_, 8, CV_RGB(255,255,0), -1);
        //! ROI
        cv::rectangle(img, roiP1_, roiP2_, CV_RGB(0,255,0), 3, 8, 0);

        //! publish drawn image.
        imgPub_.publish(br->toImageMsg());        
    }


    /*
    暂停使用
    */
    void imgCallback(const sensor_msgs::ImageConstPtr& imgMsg){
        // cout<<"image callback"<<endl;

        // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
        cv::Mat img;
        cv_bridge::CvImagePtr br;
        try{
            br = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
            img = br->image;
            // cout<<"input image size: "<<img.cols<<"  "<<img.rows<<endl;
        }catch(cv_bridge::Exception& ex){
            ROS_ERROR("Failed to convert image!");
            return;
        }

        //! draw markers on image. http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage
        cv::circle(img, center_, 10, CV_RGB(255,0,0), -1);
        //! BBX四条边的中点
        cv::circle(img, pp1PixelFrame_, 8, CV_RGB(255,0,0), -1);
        cv::circle(img, pp2PixelFrame_, 8, CV_RGB(0,255,0), -1);
        cv::circle(img, pp3PixelFrame_, 8, CV_RGB(0,0,255), -1);
        cv::circle(img, pp4PixelFrame_, 8, CV_RGB(255,255,0), -1);
        //! ROI
        cv::rectangle(img, roiP1_, roiP2_, CV_RGB(0,255,0), 3, 8, 0);

        //! publish drawn image.
        imgPub_.publish(br->toImageMsg());
    }    

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
        // cout<<"cloud callback"<<endl;
    }

    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
        cout<<"pose callback"<<endl;

        Vector3f p3;
        // p3 << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
        // cloud frame(x front) to camera frame(z front)
        p3 << -msg->pose.position.y,-msg->pose.position.z,msg->pose.position.x;
        cout<<"p3 = "<<p3<<endl;
        // Matrix3f K;
        // K << 702.67,0,636.38,
        //     0,702.67,347.11,
        //     0,0,1;

        Vector3f p2;
        // p2 = utility_.project3dTo2d(p3, K_);
        // cout<<"p2 = "<<p2<<endl<<endl;   
        // center_.x = p2(0);    
        // center_.y = p2(1);    

        p2 = utility_.project3dTo2d(p3, P_);
        cout<<"p2 = "<<p2<<endl<<endl;   
        center_.x = p2(0);    
        center_.y = p2(1);           
    }

    void bbxCallback(const jsk_recognition_msgs::BoundingBoxConstPtr& msg){
        Vector3f p3;
        // cloud frame(x front) to camera frame(z front)
        p3 << -msg->pose.position.y,-msg->pose.position.z,msg->pose.position.x;
        cout<<"p3 = "<<p3<<endl;

        Vector3f p2;  
        p2 = utility_.project3dTo2d(p3, P_);
        cout<<"p2 = "<<p2<<endl<<endl;   
        center_.x = p2(0);    
        center_.y = p2(1); 

        //! 计算BBX上下及左右边的中点（点云坐标系）
        float heightLeftCenter_z = msg->pose.position.z;
        float heightLeftCenter_y = msg->pose.position.y + 0.5*msg->dimensions.y;
        float heightLeftCenter_x = msg->pose.position.x;
        Vector3f pp1CamFrame; // 相机坐标系
        pp1CamFrame << -heightLeftCenter_y, -heightLeftCenter_z, heightLeftCenter_x;
        Vector3f pp1PixelFrame; // 图像坐标系
        pp1PixelFrame = utility_.project3dTo2d(pp1CamFrame, P_);
        pp1PixelFrame_.x = pp1PixelFrame(0);
        pp1PixelFrame_.y = pp1PixelFrame(1);

        float heightRightCenter_z = msg->pose.position.z;
        float heightRightCenter_y = msg->pose.position.y - 0.5*msg->dimensions.y;
        float heightRightCenter_x = msg->pose.position.x;
        Vector3f pp2CamFrame; // 相机坐标系
        pp2CamFrame << -heightRightCenter_y, -heightRightCenter_z, heightRightCenter_x;
        Vector3f pp2PixelFrame; // 图像坐标系
        pp2PixelFrame = utility_.project3dTo2d(pp2CamFrame, P_);
        pp2PixelFrame_.x = pp2PixelFrame(0);
        pp2PixelFrame_.y = pp2PixelFrame(1);

        float widthTopCenter_z = msg->pose.position.z + 0.5*msg->dimensions.z;
        float widthTopCenter_y = msg->pose.position.y;
        float widthTopCenter_x = msg->pose.position.x;
        Vector3f pp3CamFrame; // 相机坐标系
        pp3CamFrame << -widthTopCenter_y, -widthTopCenter_z, widthTopCenter_x;
        Vector3f pp3PixelFrame; // 图像坐标系
        pp3PixelFrame = utility_.project3dTo2d(pp3CamFrame, P_);
        pp3PixelFrame_.x = pp3PixelFrame(0);
        pp3PixelFrame_.y = pp3PixelFrame(1);

        float widthBottomCenter_z = msg->pose.position.z - 0.5*msg->dimensions.z;
        float widthBottomCenter_y = msg->pose.position.y;
        float widthBottomCenter_x = msg->pose.position.x;
        Vector3f pp4CamFrame; // 相机坐标系
        pp4CamFrame << -widthBottomCenter_y, -widthBottomCenter_z, widthBottomCenter_x;
        Vector3f pp4PixelFrame; // 图像坐标系
        pp4PixelFrame = utility_.project3dTo2d(pp4CamFrame, P_);
        pp4PixelFrame_.x = pp4PixelFrame(0);
        pp4PixelFrame_.y = pp4PixelFrame(1);

        //! ROI(图像坐标系)
        roiP1_.x = pp1PixelFrame_.x;
        roiP1_.y = pp3PixelFrame_.y;

        roiP2_.x = pp2PixelFrame_.x;
        roiP2_.y = pp4PixelFrame_.y;    
    }

private:
    Utility utility_;

    cv::Point2d center_;
    cv::Point2d pp1PixelFrame_, pp2PixelFrame_, pp3PixelFrame_, pp4PixelFrame_;
    cv::Point2i roiP1_, roiP2_;

    // # Intrinsic camera matrix for the raw (distorted) images.
    // #     [fx  0 cx]
    // # K = [ 0 fy cy]
    // #     [ 0  0  1]
    // # Projects 3D points in the camera coordinate frame to 2D pixel
    // # coordinates using the focal lengths (fx, fy) and principal point
    // # (cx, cy).    
    Matrix3f K_;  

    MatrixXf P_;
};


int main(int argc, char** argv){
    ros::init(argc, argv, "registration");

    ros::NodeHandle node("~");

    cout<<"Registration node started."<<endl;
    Registration registration(node);
    // registration.initilization();
    
    ros::spin();
    cout<<"Registration node end."<<endl;

    return 0;
}
