#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
/* 
    http://wiki.ros.org/cv_bridge/
    http://wiki.ros.org/image_transport/Tutorials/PublishingImages
    camera_info:
    https://blog.csdn.net/weixin_40830684/article/details/102214481
    https://answers.ros.org/question/278602/how-to-use-camera_info_manager-to-publish-camera_info/
    https://github.com/ros-drivers/camera1394/blob/master/src/nodes/driver1394.cpp#L346
 */
 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_to_rosmsg");
    ros::NodeHandle node;

    std::cout<<"Use recorded video for detection."<<std::endl;

    // load video file.
    cv::Mat image_;
    // std::string videoName_ = "/home/bluerov2/bluerov2/src/pcl/video/a1.mp4";
    std::string videoName_ = argv[1];
    cv::VideoCapture capture;
    image_ = cv::imread(videoName_, cv::IMREAD_COLOR);
    if (image_.empty())
    {
        if (!capture.open(videoName_))
        {
            cout << "Could not read " << videoName_ << endl;
            return 1;
        }
    }        
    std::cout<<"Vedio loaded."<<std::endl;
    std::cout<<"image width: "<<image_.cols<<"  "<<"height: "<<image_.rows<<std::endl;


    // publish video rosmsg.
    image_transport::ImageTransport it_(node);
    image_transport::CameraPublisher imgPub_; //Publisher imgPub_;
    // imgPub_ = it_.advertise("/zed/zed_node/left/image_rect_color", 30);
    imgPub_ = it_.advertiseCamera("/zed/zed_node/left/image_rect_color", 30);

    // camera_info
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    std::string camera_name_ = "vitural_camera";
    std::string cam_info_url_ = argv[2]; //"/home/bluerov/data/camera_info.yaml";
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node, camera_name_, cam_info_url_));
    sensor_msgs::CameraInfoPtr ci_(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));


    ros::Rate rate(10);
    while(node.ok()){
        capture >> image_;

        if(!image_.empty()){

            // transform to rosmsg.
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
            ci_->header.frame_id = msg->header.frame_id;
            ci_->header.stamp = msg->header.stamp;                
            imgPub_.publish(*msg, *ci_);             
            cv::waitKey(1);
        }
   
        ros::spinOnce();
        rate.sleep();
    }

    return 0;    
}