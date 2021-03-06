#include "CascadeDetection.hpp"

using namespace cv;
using namespace std;


int rawImageW, rawImageH; // 原始输入图像的长宽

cv::Rect confirmedRect; // 用来保存最终被成功检测到的目标rect
cv::Rect confirmedRectPrevious; // 保存上一次成功检测到的目标
// cv::Point2d confirmedRectPreviousPoint, confirmedRectCurrentPoint;

// 以下参数用于处理目标的跳变和丢失：
bool firstDetected; //首次检测到标志，用于处理跳变和丢失
int failedDetectCount; // 目标丢失帧数
int failedDetectTotalCount; // 连续不间断统计的目标丢失帧数（不会复位成0）
int failedDetectMaxCount; // 允许目标丢失帧数的最大阈值
int subscribedImageCount; // 视频帧数统计
cv::Point2d successiveRectBias; // 前后两帧目标位置变化量
double rectMaxBiasX, rectMaxBiasY; //允许的前后帧目标位置变化量阈值
int jumpBiasCount; // 连续跳变的帧数
int jumpBiasTotalCount; // 连续跳变的帧数（不会复位成0）
int jumpBiasCountMax; //允许连续跳变的帧数的最大阈值

// 统计检测用时
double globalTime, nestedTime; // 全局检测器用时，局部检测器用时

bool useVideo; // 是否使用录制的视频作为输入源（用于评估图像目标检查）


namespace bluerov_detection_tracking
{
    CascadeDetection::CascadeDetection(ros::NodeHandle& node)
        : node_(node),
        it_(node)
    {
        cout<<"imgCloudCascadeDetection"<<endl;

        readParameters();

        initilization();

        // ============================ 订阅与检测
        string imgTopic = node_.resolveName("/zed/zed_node/left/image_rect_color");
        // string cloudTopic = node_.resolveName("/zed/zed_node/point_cloud/cloud_registered");
        // string poseTopic = node_.resolveName("/republish_cxx_node/bbxpose");
        string bbx3dTopic = node_.resolveName("/cloud_detection/bbx");

        if(imageAndCameraInfoCallback_)
            imgCamSub_ = it_.subscribeCamera(imgTopic, 30, &CascadeDetection::imgCamCallback, this);
        else
            imgSub_ = it_.subscribe(imgTopic, 30, &CascadeDetection::imgCallback, this);
        

        // 以下为测试用：----------------------------------------------------
        // cloudSub_ = node_.subscribe(cloudTopic, 10, &CascadeDetection::cloudCallback, this);
        // poseSub_ = node_.subscribe(poseTopic, 10, &CascadeDetection::poseCallback, this);
        // bbx3dSub_ = node_.subscribe(bbx3dTopic, 10, &CascadeDetection::bbx3dCallback, this);
        // ----------------------------------------------------

        // ============================ 发布结果
        reprojected3DBBXcenterMarkerPub_ = node_.advertise<visualization_msgs::Marker>("reprojected_3dbbx_center", 20);
        frustum_pub = node_.advertise<visualization_msgs::Marker>("frustum", 30);
        trapezoid3D_pub = node_.advertise<visualization_msgs::Marker>("trapezoid3D", 30);
        rovRect_pub = node_.advertise<jsk_recognition_msgs::Rect>("rov_rect", 30);
        // http://wiki.ros.org/image_transport/Tutorials/PublishingImages
        imgPub_ = it_.advertise("image", 30);

    }


    CascadeDetection::~CascadeDetection(){
        node_.shutdown();
    }


    bool CascadeDetection::readParameters(){
        ROS_INFO("Loading parameters from parameter server ......");

        node_.param("debug", debug_, bool(false));

        node_.param("cascadeName", cascadeName_, string("/home/bluerov2/bluerov2/src/pcl/cascade/cascade2_big.xml"));
        node_.param("nestedCascadeName", nestedCascadeName_, string("/home/bluerov2/bluerov2/src/pcl/cascade/cascade4_small.xml"));
        node_.param("videoName", videoName_, string("/home/bluerov2/bluerov2/src/pcl/video/a1.mp4"));

        node_.param("detectMultiScale/minSizeScale", minSizeScale_, 0.1);
        node_.param("detectMultiScale/maxSizeScale", maxSizeScale_, 1.0);
        node_.param("detectMultiScale/continuousDetectNum", continuousDetectNum_, int(10));

        node_.param("detectMultiScale/minColSizeScaleNested", minColSizeScaleNested_, 0.1);
        node_.param("detectMultiScale/minRowSizeScaleNested", minRowSizeScaleNested_, 0.1);
        node_.param("detectMultiScale/maxColSizeScaleNested", maxColSizeScaleNested_, 0.8);
        node_.param("detectMultiScale/maxRowSizeScaleNested", maxRowSizeScaleNested_, 0.8);
        node_.param("detectMultiScale/continuousDetectNumNested", continuousDetectNumNested_, int(5));

        node_.param("rectNestedScale", rectNestedScale_, 1.0);
        node_.param("scale", scale_, 1.0);

        node_.param("lost_jump/failedDetectMaxCount", failedDetectMaxCount, int(20));
        node_.param("lost_jump/jumpBiasCountMax", jumpBiasCountMax, int(10));

        node_.param("imageAndCameraInfoCallback", imageAndCameraInfoCallback_, bool(true));

        node_.param("filterDepthValue", filterDepthValue_, 5.0);
        
        ROS_INFO("Done.");
        return true;
    }


    bool CascadeDetection::initilization(){
        ROS_INFO("Initilization ......");

        //
        subscribedImageCount = 0;
        // failedDetectMaxCount = 20;
        failedDetectCount = 0;
        failedDetectTotalCount = 0;

        //
        jumpBiasCount = 0;
        // jumpBiasCountMax = 10;
        jumpBiasTotalCount = 0;

        firstDetected = true;
        rectMaxBiasX = rectMaxBiasY = 100;   

        // 
        globalTime = 0;
        nestedTime = 0;     

        useVideo = true;

        //! 加载cascade目标模型
        if (!nestedCascade_.load(nestedCascadeName_))
            cerr << "WARNING: Could not load classifier cascade for nested objects" << endl;
        std::cout<<"Nested cascade loaded."<<std::endl;
        if (!cascade_.load(cascadeName_))
        {
            cerr << "ERROR: Could not load classifier cascade" << endl;
            return -1;
        }
        std::cout<<"Global cascade loaded."<<std::endl;


        /* if(useVideo){
            std::cout<<"Use recorded video for detection."<<std::endl;
            image_ = imread(videoName_, IMREAD_COLOR);
            if (image_.empty())
            {
                if (!capture.open(videoName_))
                {
                    cout << "Could not read " << videoName_ << endl;
                    return 1;
                }
            }        
            std::cout<<"Vedio loaded."<<std::endl;
        } */



        //! 初始化参数变量
        tryflip_ = false;


        /*         // 初始化卡尔曼滤波器
        // motion model: constant velocity, (x,y,vx,vy)
        kf.x_ = Eigen::VectorXd(4);
        kf.x_ << 0, 0, 0, 0, 0;

        kf.F_ = Eigen::MatrixXd(4,4);
        kf.F_ << 1,0,1,0,
                 0,1,0,1,
                 0,0,1,0,
                 0,0,0,1;

        kf.P_ = Eigen::MatrixXd(4,4);
        kf.P_ << 1,0,0,0,
                 0,1,0,0,
                 0,0,1,0,
                 0,0,0,1;

        kf.H_ = Eigen::MatrixXd(2,4);
        kf.H_ << 1,0,0,0,
                 0,1,0,0;    

        kf.Q_ = Eigen::MatrixXd(4,4);

        kf.R_ = Eigen::MatrixXd(2,2);
        kf.R_ << kalmanFilaterPara.xNoise, 0,
                 0, kalmanFilaterPara.yNoise;         

        // previousTime_ =      */



        ROS_INFO("Done.");     
        std::cout<<"Waiting for video stream ......"<<std::endl;   
        return true;
    }    


    void CascadeDetection::imgCamCallback(  const sensor_msgs::ImageConstPtr& imgMsg, 
                                            const sensor_msgs::CameraInfoConstPtr& infoMsg){
        cout<<"IMAGE SUBSCRIBE MODE: image AND camera_info callback"<<endl;

        ++subscribedImageCount;

        // ========================================================================================
        //! 获取相机的内参
        K_ << infoMsg->K[0],0,infoMsg->K[2],
            0,infoMsg->K[4],infoMsg->K[5],
            0,0,1;         
          
        P_.resize(3,4);  
        P_ << infoMsg->P[0],0,infoMsg->P[2],infoMsg->P[3],
            0,infoMsg->P[5],infoMsg->P[6],infoMsg->P[7],
            0,0,1,0;    

        camModel_.fromCameraInfo(infoMsg);
        // cout<<"camera model: "<<camModel_<<endl;
        // ================================= END =================================================


        // ========================================================================================
        //! 获取RGB图像，转成opencv格式，并处理后发布出去
        // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
        // cv::Mat image_;
        cv_bridge::CvImagePtr br;
        try{
            br = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
            image_ = br->image;
            // cout<<"input image size: "<<image_.cols<<"  "<<image_.rows<<endl;
        }catch(cv_bridge::Exception& ex){
            ROS_ERROR("Failed to convert image!");
            return;
        }
        // ================================= END =================================================


        // ========================================================================================
        // 开始检测
        rectMaxBiasX = rectMaxBiasY = rawImageW*0.1;        
        confirmedObjectFlag_ = false;
        confirmedROV_ = detect(image_, cascade_, nestedCascade_, scale_);
        // ================================= END =================================================


        // ========================================================================================
        // 检测结果
        jsk_recognition_msgs::Rect rovRect;
        if(confirmedObjectFlag_) // 检测成功
        {
            failedDetectCount = 0;

            //! 将目标检测结果（2D BBX）发布出去，将被3D点云detector节点订阅
            rovRect.x = confirmedROV_.x;
            rovRect.y = confirmedROV_.y;
            rovRect.width = confirmedROV_.width;
            rovRect.height = confirmedROV_.height;

            // 只有当成功检测出目标才把目标rect发布出去，这样就方便订阅了该话题的节点不用去判断该目标rect是否有效了（长宽高不为0）
            if(confirmedObjectFlag_){
                rovRect_pub.publish(rovRect);

            }

            // 以下代码为测试用#####################################################################
            //! 将目标2d bbx的4个顶点转换到相机坐标系下
            bool project2dto3d = false;
            if(project2dto3d){
                Vector2f v1Pixel, v2Pixel, v3Pixel, v4Pixel; // 图像坐标系下的4个顶点，顺时针，左上角到左下角依次为1/2/3/4
                v1Pixel(0) = confirmedROV_.x;
                v1Pixel(1) = confirmedROV_.y;
                v2Pixel(0) = confirmedROV_.x + confirmedROV_.width;
                v2Pixel(1) = confirmedROV_.y;    
                v3Pixel(0) = confirmedROV_.x + confirmedROV_.width;
                v3Pixel(1) = confirmedROV_.y + confirmedROV_.height;    
                v4Pixel(0) = confirmedROV_.x;
                v4Pixel(1) = confirmedROV_.y + confirmedROV_.height;                    
                Vector3f v1Cam, v2Cam, v3Cam, v4Cam; // 相机坐标系下的4个顶点，顺时针，左上角到左下角依次为1/2/3/4
                float range = 1;
                v1Cam = utility_.project2dTo3d(v1Pixel, K_, range);
                v2Cam = utility_.project2dTo3d(v2Pixel, K_, range);
                v3Cam = utility_.project2dTo3d(v3Pixel, K_, range);
                v4Cam = utility_.project2dTo3d(v4Pixel, K_, range);
                Vector3f v1Cam2, v2Cam2, v3Cam2, v4Cam2; // 相机坐标系下的4个顶点，顺时针，左上角到左下角依次为1/2/3/4
                float range2 = 3;
                v1Cam2 = utility_.project2dTo3d(v1Pixel, K_, range2);
                v2Cam2 = utility_.project2dTo3d(v2Pixel, K_, range2);
                v3Cam2 = utility_.project2dTo3d(v3Pixel, K_, range2);
                v4Cam2 = utility_.project2dTo3d(v4Pixel, K_, range2);      

                //! 绘制梯形锥体--------------------------------------
                //! 梯形椎体，共有8个顶点，12条边，通过绘制12条线marker方式实现在rviz中的显示
                //! 顶点ID：从顶面开始到底面、面四边从左上角开始顺时针。1//2/3/4/5/6/7/8
                std::vector<geometry_msgs::Point> trapezoid3DLine;
                std::vector< std::vector<geometry_msgs::Point> > trapezoid3DAllLine;
                geometry_msgs::Point v1, v2, v3, v4, v5, v6, v7, v8; // ZED相机的点云坐标系为前左上坐标系
                // geometry_msgs
                // trapezoid3D_pub.publish();
                //! 顶面4条边
                v1.x = range;
                v1.y = -v1Cam(0);
                v1.z = -v1Cam(1);    
                v2.x = range;
                v2.y = -v2Cam(0);
                v2.z = -v2Cam(1);              
                trapezoid3DLine.push_back(v1);
                trapezoid3DLine.push_back(v2);
                trapezoid3DAllLine.push_back(trapezoid3DLine);         
                
                v3.x = range;
                v3.y = -v3Cam(0);
                v3.z = -v3Cam(1);            
                trapezoid3DLine.push_back(v2);
                trapezoid3DLine.push_back(v3);
                trapezoid3DAllLine.push_back(trapezoid3DLine); 

                v4.x = range;
                v4.y = -v4Cam(0);
                v4.z = -v4Cam(1);              
                trapezoid3DLine.push_back(v3);
                trapezoid3DLine.push_back(v4);
                trapezoid3DAllLine.push_back(trapezoid3DLine);      

                trapezoid3DLine.push_back(v4);
                trapezoid3DLine.push_back(v1);
                trapezoid3DAllLine.push_back(trapezoid3DLine);                
                //! 底面4条边
                v5.x = range2;
                v5.y = -v1Cam2(0);
                v5.z = -v1Cam2(1);    
                v6.x = range2;
                v6.y = -v2Cam2(0);
                v6.z = -v2Cam2(1);              
                trapezoid3DLine.push_back(v5);
                trapezoid3DLine.push_back(v6);
                trapezoid3DAllLine.push_back(trapezoid3DLine);         
                
                v7.x = range2;
                v7.y = -v3Cam2(0);
                v7.z = -v3Cam2(1);            
                trapezoid3DLine.push_back(v6);
                trapezoid3DLine.push_back(v7);
                trapezoid3DAllLine.push_back(trapezoid3DLine); 

                v8.x = range2;
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
                double width(0.02);                
                utility_.publish3DConvexHullMarkers("zed_left_camera_frame", color, width, trapezoid3DAllLine, trapezoid3D_pub);
            }
            //###################################################################################

            //! 在图像上绘制2D目标检测结果方框. http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage
            cv::circle(image_, center_, 10, CV_RGB(255,0,0), -1);
            // 绘制方框
            cv::rectangle(image_, roiP1_, roiP2_, CV_RGB(0,255,0), 3, 8, 0);
            // 绘制方框四条边的中点
            cv::circle(image_, pp1PixelFrame_, 8, CV_RGB(255,0,0), -1);
            cv::circle(image_, pp2PixelFrame_, 8, CV_RGB(0,255,0), -1);
            cv::circle(image_, pp3PixelFrame_, 8, CV_RGB(0,0,255), -1);
            cv::circle(image_, pp4PixelFrame_, 8, CV_RGB(255,255,0), -1);            
        }
        else // 检测失败
        {
            failedDetectCount++;
            failedDetectTotalCount++;
            cv::putText(image_, std::to_string(failedDetectCount), 
                cv::Point(5, std::floor(0.95*image_.rows)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 1, 8, false);

            cv::putText(image_, "Failed Detect!", 
                cv::Point(100,std::floor(0.9*image_.rows)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1, 8, false);

            if(failedDetectCount <= failedDetectMaxCount){
                rovRect.x = confirmedROV_.x;
                rovRect.y = confirmedROV_.y;
                rovRect.width = confirmedROV_.width;
                rovRect.height = confirmedROV_.height;
            }

            if(failedDetectCount > failedDetectMaxCount){
                cv::putText(image_, "Failed Detect Max count reached, set rect to the whole image window!", 
                    cv::Point(100,std::floor(0.95*image_.rows)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1, 8, false);    

                rovRect.x = 10;
                rovRect.y = 10;
                rovRect.width = std::floor(rawImageW*0.8);
                rovRect.height = std::floor(rawImageH*0.8);                
            }

            rovRect_pub.publish(rovRect);
        }


        // 发布图像
        imgPub_.publish(br->toImageMsg());     


        // 打印图像目标检测的性能：
        // (1)准确率 = 1- (漏检帧数+跳变帧数/总图像帧数
        std::cout<<"total frames = "<<subscribedImageCount<<"  lost number = "<<failedDetectTotalCount<<"  jump number = "<<jumpBiasTotalCount<<endl;
        double accuracy = 0;
        accuracy = (1 - 1.0*(failedDetectTotalCount+jumpBiasTotalCount)/subscribedImageCount)*100; // 必须乘以1.0将整数转换为浮点数，否则结果出错
        std::cout<<"Accuracy = "<<accuracy<<endl;
        // (2)平均检测用时
        double globalMeanTime = globalTime/subscribedImageCount;
        double nestedMeanTime = nestedTime/subscribedImageCount;
        std::cout<<"global detetor MeanTime(ms) = "<<globalMeanTime<<"  nested detetor MeanTime(ms) = "<<nestedMeanTime<<endl;
        std::cout<<"################################################"<<endl;
        std::cout<<endl<<endl;
        // ================================= END =================================================

        /* 
        // 卡尔曼滤波，滤波后的2D目标box中心：蓝色
        // predict
        if(debug_) cout<<"Prediction ..."<<endl;
        kf.Predict();
        if(debug_) cout<<"predicted x_ = "<<endl<<kf.x_<<endl;
        
        // measurement update
        if(debug_) cout<<"Update ..."<<endl;
        kf.Update(measurements);        
        if(debug_){
            cout << "fused x_= " << endl<<kf.x_ << endl;
            cout << "P_= " << endl<<kf.P_ << endl;  
        } 

        // 绘制融合后的目标中心：蓝色
        cv::Point2d box2dCenterFused;
        box2dCenterFused.x = kf.x_(0);
        box2dCenterFused.y = kf.x_(1);
        cv::circle(image_, box2dCenterFused, 10, CV_RGB(0,0,255), -1);
        */
    }


    cv::Rect CascadeDetection::detect(  Mat& image, 
                                        CascadeClassifier& cascade,
                                        CascadeClassifier& nestedCascade,
                                        double scale )
    {
        double t = 0;
        vector<Rect> bluerovs;
        Mat gray, smallImg;

        cv::Size minSize, maxSize;
        minSize = Size(smallImg.cols*minSizeScale_, smallImg.rows*minSizeScale_);
        maxSize = Size(smallImg.cols*maxSizeScale_, smallImg.rows*maxSizeScale_);   

        //! 图像预处理：灰度化、调整尺寸、直方图均衡化
        cvtColor( image, gray, COLOR_BGR2GRAY );
        double fx = 1 / scale;
        resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR_EXACT );
        equalizeHist( smallImg, smallImg );

        // 
        rawImageW = image.cols;
        rawImageH = image.rows;
        cout<<"image  w,h: "<<image.cols<<" "<<image.rows<<endl;
        cout<<"smallimg  w,h: "<<smallImg.cols<<" "<<smallImg.rows<<endl;


        //! ----------------------- 第1轮检测：整个目标检测器 -----------------------
        cout<<"============== 1st round detection ===================="<<endl;
        t = (double)getTickCount();
        cascade.detectMultiScale(   smallImg, 
                                    bluerovs,
                                    1.1, continuousDetectNum_, 0
                                    |CASCADE_FIND_BIGGEST_OBJECT,
                                    //|CASCADE_DO_ROUGH_SEARCH
                                    // |CASCADE_SCALE_IMAGE,
                                    minSize, maxSize);
        t = (double)getTickCount() - t;
        if(debug_) printf( "Time = %g ms\n", t*1000/getTickFrequency());
        globalTime += t*1000/getTickFrequency(); // 累计用时


        // cv::Rect confirmedRect; // 用来保存最终被成功检测到的目标rect
        for ( size_t i = 0; i < bluerovs.size(); i++ )
        {
            //! 整体目标
            Rect rect = bluerovs[i];
            if(debug_){
                cout<<"rect origin:  "<<rect.x<<"  "<<rect.y<<endl;
                cout<<"rect w h:  "<<rect.width<<"  "<<rect.height<<endl;
            }

            //! 局部目标
            cout<<"----------- 2nd round detection ----------------"<<endl;
            Rect rectNestedScaled;
            rectNestedScaled = rect;

            rectNestedScaled.width = rect.width*rectNestedScale_;
            rectNestedScaled.height = rect.height*1; 
            float rCenter = rect.x + rect.width/2;      
            rectNestedScaled.x = rCenter - rectNestedScaled.width/2;      
            if(debug_){
                cout<<"rectNestedScaled origin:  "<<rectNestedScaled.x<<"  "<<rectNestedScaled.y<<endl;
                cout<<"rectNestedScaled w h:  "<<rectNestedScaled.width<<"  "<<rectNestedScaled.height<<endl;
                cout<<"r22  origin: "<<rectNestedScaled.x<<"  "<<rectNestedScaled.y<<endl;
                cout<<"r22 w h:  "<<rectNestedScaled.width<<"  "<<rectNestedScaled.height<<endl;   
                // cout<<"rect center = "<<rCenter<<endl;
                // cout<<"rectNestedScaled width/2 = "<<rectNestedScaled.width*0.5<<endl;                           
            }

            Scalar color1 = Scalar(0,255,0); // 绿色
            // rectangle( image, Point(cvRound(rect.x*scale), cvRound(rect.y*scale)),
            //             Point(cvRound((rect.x + rect.width-1)*scale), cvRound((rect.y + rect.height-1)*scale)),
            //             color1, 3, 8, 0);
            rectangle( image, Point(cvRound(rectNestedScaled.x*scale), cvRound(rectNestedScaled.y*scale)),
                        Point(cvRound((rectNestedScaled.x + rectNestedScaled.width-1)*scale), cvRound((rectNestedScaled.y + rectNestedScaled.height-1)*scale)),
                        color1, 3, 8, 0);                    
            cout<<endl;

            if( nestedCascade.empty() )
                continue;


            //! ----------------------- 第2轮检测：嵌套检测器（局部ROI特征）-----------------------
            vector<Rect> bluerovsNested;    
            Mat smallImgROI;
            smallImgROI = smallImg( rectNestedScaled );
            cout<<"smallImgROI w,h: "<<smallImgROI.cols<<" "<<smallImgROI.rows<<endl;
            Size minSizeNested = Size(smallImgROI.cols*minColSizeScaleNested_, smallImgROI.rows*minRowSizeScaleNested_);
            Size maxSizeNested = Size(smallImgROI.cols*maxColSizeScaleNested_, smallImgROI.rows*maxRowSizeScaleNested_);  
            //

            t = (double)getTickCount();
            nestedCascade.detectMultiScale( smallImgROI, 
                                            bluerovsNested,
                                            1.1, continuousDetectNumNested_, 0
                                            |CASCADE_FIND_BIGGEST_OBJECT,
                                            //|CASCADE_DO_ROUGH_SEARCH
                                            //|CASCADE_DO_CANNY_PRUNING
                                            // |CASCADE_SCALE_IMAGE,
                                            minSizeNested, maxSizeNested );
            t = (double)getTickCount() - t;
            if(debug_) printf( "Time = %g ms\n", t*1000/getTickFrequency());
            nestedTime += t*1000/getTickFrequency(); //累计用时


            // 被检测到的局部目标
            // for ( size_t j = 0; j < bluerovsNested.size(); j++ )
            if(bluerovsNested.size() > 0) // 内层嵌套检测器只允许一次检测到一个目标(只保留bbx最大的目标)
            {
                // Rect rectNested = bluerovsNested[j];
                Rect rectNested = bluerovsNested[0];
                Scalar colorNested = Scalar(255,0,0);  //蓝色，BGR     

                rectangle(  image, 
                            Point(cvRound((rectNestedScaled.x+rectNested.x)*scale), cvRound((rectNestedScaled.y+rectNested.y)*scale)),
                            Point(cvRound((rectNestedScaled.x + rectNested.width-1)*scale), cvRound((rectNestedScaled.y + rectNested.height-1)*scale)),
                            colorNested, 3, 8, 0);   

                confirmedObjectFlag_ = true;                           
            }
            cout<<"----------- /2nd round detection ----------------"<<endl;

            
            // 最终确定被成功检测到的目标
            if(confirmedObjectFlag_){
                Scalar colorFused = Scalar(0,0,255);  //红色，BGR     
                rectangle( image, Point(cvRound(rect.x*scale), cvRound(rect.y*scale)),
                            Point(cvRound((rect.x + rect.width-1)*scale), cvRound((rect.y + rect.height-1)*scale)),
                            colorFused, 3, 8, 0); 

                cout<<"============== /1st round detection ===================="<<endl;

                confirmedRect = rect;

                // 跳变处理
                if(firstDetected){
                    successiveRectBias.x = 0 ;
                    successiveRectBias.y = 0 ;
                }else{
                    successiveRectBias.x = confirmedRect.x - confirmedRectPrevious.x;
                    successiveRectBias.y = confirmedRect.y - confirmedRectPrevious.y;
                }
                rectMaxBiasX = rectMaxBiasY = confirmedRect.width;
                if(abs(successiveRectBias.x)>rectMaxBiasX || abs(successiveRectBias.y)>rectMaxBiasY){
                    ++jumpBiasCount;
                    ++jumpBiasTotalCount;
                    if(jumpBiasCount > jumpBiasCountMax){
                        firstDetected = true;
                    }
                    ROS_ERROR("Current detect object is too far away from the previous detect object!");
                    std::cout<<"successiveRectBias.x= "<<successiveRectBias.x<<"  successiveRectBias.y= "<<successiveRectBias.y<<std::endl;
                    confirmedRect = confirmedRectPrevious; // 跳变之前的目标位置作为本次检测输出
                    return confirmedRect;
                }
                jumpBiasCount = 0;
                
                confirmedRectPrevious = confirmedRect;

                firstDetected = false;

                return confirmedRect;
                // break; // 假设1帧图像中只存在1个目标
            }else{
                confirmedRect = rect;                
            }                
        }

        if(debug_) ROS_WARN("No object is detected!");
        cout<<"============== /1st round detection ===================="<<endl;
        
        //! 未检测到目标情况下的返回，此时rect的宽和高都为0
        // confirmedRect.width = confirmedRect.height = 0.0;

        firstDetected = false;

        return confirmedRect;        
    }


    /* 
    void CascadeDetection::bbx3dCallback(const jsk_recognition_msgs::BoundingBoxConstPtr& msg){
        cout<<endl<<endl;

        Vector3f p3;
        // cloud frame(x front) to camera frame(z front)
        p3 << -msg->pose.position.y,-msg->pose.position.z,msg->pose.position.x;
        cout<<"3D BBX position(cloud frame) p3_1 = "<<p3<<endl;

        //! 3d投影到2d
        Vector3f p2;  
        p2 = utility_.project3dTo2d(p3, P_);
        cout<<"2D BBX position projrected from 3D BBX(image frame) p2 = "<<p2<<endl<<endl;   
        center_.x = p2(0);    
        center_.y = p2(1); 

        //! 2d投影到3d ----------------------------
        //! 使用函数1，有误差
        Vector3f p3_2;
        Vector2f p22;
        p22(0) = p2(0);
        p22(1) = p2(1);
        float depthScale = 1.0;
        float depth = p3(2);
        p3_2 = utility_.project2dTo3d(p22, K_, depthScale, depth);
        center3d_.x = p3_2(0);
        center3d_.y = p3_2(1);
        center3d_.z = p3_2(2);
        cout<<"[1]3D BBX position projrected from 2D BBX(image frame),should be equal to p3_1, p3_2 = "<<p3_2<<endl<<endl;   
        //! 使用函数2，没有误差
        p3_2 = utility_.project2dTo3d(p22, K_, depth);
        center3d_.x = p3_2(0);
        center3d_.y = p3_2(1);
        center3d_.z = p3_2(2);
        cout<<"[2]3D BBX position projrected from 2D BBX(image frame),should be equal to p3_1, p3_2 = "<<p3_2<<endl<<endl;  

        visualization_msgs::Marker center3dMarker;
        center3dMarker.type = visualization_msgs::Marker::SPHERE;   
        center3dMarker.action = visualization_msgs::Marker::ADD;
        center3dMarker.lifetime = ros::Duration(0);
        center3dMarker.header.frame_id = "zed_left_camera_optical_frame"; 
        // center3dMarker.header.frame_id = "zed_camera_center"; 
        center3dMarker.color.r = 255;        
        center3dMarker.color.a = 1;
        center3dMarker.scale.x = center3dMarker.scale.y = center3dMarker.scale.z = 0.05;
        center3dMarker.pose.orientation.w = 1.0;    
        center3dMarker.pose.position.x = center3d_.x; 
        center3dMarker.pose.position.y = center3d_.y; 
        center3dMarker.pose.position.z = center3d_.z;  
        reprojected3DBBXcenterMarkerPub_.publish(center3dMarker);


        //! 绘制锥体--------------------------------------
        //! 底面为矩形的椎体，共有5个顶点，8条边，通过绘制8条线marker方式实现在rviz中的显示
        //! 顶点ID：锥体尖顶0、锥体底面四边从左上角开始顺时针：左上角1、右上角2、右下角3、左下角4
        std::vector<geometry_msgs::Point> frustumLine;
        std::vector< std::vector<geometry_msgs::Point> > frutumAllLine;
        geometry_msgs::Point v0, v1, v2, v3, v4;
        //! 竖的4条边
        v0.x = v0.y = v0.z = 0;
        v1.x = filterDepthValue_;
        v1.y = 1;
        v1.z = 2;        
        frustumLine.push_back(v0);
        frustumLine.push_back(v1);
        frutumAllLine.push_back(frustumLine); 

        v2.x = filterDepthValue_;
        v2.y = -1;
        v2.z = 2;        
        frustumLine.push_back(v0);
        frustumLine.push_back(v2);
        frutumAllLine.push_back(frustumLine);  

        v3.x = filterDepthValue_;
        v3.y = -1;
        v3.z = -2;        
        frustumLine.push_back(v0);
        frustumLine.push_back(v3);     
        frutumAllLine.push_back(frustumLine);  

        v4.x = filterDepthValue_;
        v4.y = 1;
        v4.z = -2;        
        frustumLine.push_back(v0);
        frustumLine.push_back(v4);            
        frutumAllLine.push_back(frustumLine);  
        //! 底面的4条边
        frustumLine.push_back(v1);
        frustumLine.push_back(v4);            
        frutumAllLine.push_back(frustumLine);  
        frustumLine.push_back(v4);
        frustumLine.push_back(v3);            
        frutumAllLine.push_back(frustumLine); 
        frustumLine.push_back(v3);
        frustumLine.push_back(v2);            
        frutumAllLine.push_back(frustumLine);  
        frustumLine.push_back(v2);
        frustumLine.push_back(v1);            
        frutumAllLine.push_back(frustumLine);   
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        double width(0.01);                
        utility_.publish3DConvexHullMarkers("zed_left_camera_frame", color, width, frutumAllLine, frustum_pub);



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
    */
    

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
        // cout<<"cloud callback"<<endl;
    }



    void CascadeDetection::imgCallback(  const sensor_msgs::ImageConstPtr& imgMsg){
        cout<<endl<<endl;
        cout<<"IMAGE SUBSCRIBE MODE:  image ONLY callback"<<endl;

        ++subscribedImageCount;



        // ========================================================================================
        //! 获取RGB图像，转成opencv格式，并处理后发布出去
        // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
        // cv::Mat image_;
        cv_bridge::CvImagePtr br;
        try{
            br = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
            image_ = br->image;
            // cout<<"input image size: "<<image_.cols<<"  "<<image_.rows<<endl;
        }catch(cv_bridge::Exception& ex){
            ROS_ERROR("Failed to convert image!");
            return;
        }
        // ================================= END =================================================


        // ========================================================================================
        // 开始检测
        rectMaxBiasX = rectMaxBiasY = rawImageW*0.1;        
        confirmedObjectFlag_ = false;
        confirmedROV_ = detect(image_, cascade_, nestedCascade_, scale_);
        // ================================= END =================================================


        // ========================================================================================
        // 检测结果
        jsk_recognition_msgs::Rect rovRect;
        if(confirmedObjectFlag_) // 检测成功
        {
            failedDetectCount = 0;

            //! 将目标检测结果（2D BBX）发布出去，将被3D点云detector节点订阅
            rovRect.x = confirmedROV_.x;
            rovRect.y = confirmedROV_.y;
            rovRect.width = confirmedROV_.width;
            rovRect.height = confirmedROV_.height;

            // 只有当成功检测出目标才把目标rect发布出去，这样就方便订阅了该话题的节点不用去判断该目标rect是否有效了（长宽高不为0）
            if(confirmedObjectFlag_){
                rovRect_pub.publish(rovRect);

            }

            // 以下代码为测试用#####################################################################
            //! 将目标2d bbx的4个顶点转换到相机坐标系下
            bool project2dto3d = false;
            if(project2dto3d){
                Vector2f v1Pixel, v2Pixel, v3Pixel, v4Pixel; // 图像坐标系下的4个顶点，顺时针，左上角到左下角依次为1/2/3/4
                v1Pixel(0) = confirmedROV_.x;
                v1Pixel(1) = confirmedROV_.y;
                v2Pixel(0) = confirmedROV_.x + confirmedROV_.width;
                v2Pixel(1) = confirmedROV_.y;    
                v3Pixel(0) = confirmedROV_.x + confirmedROV_.width;
                v3Pixel(1) = confirmedROV_.y + confirmedROV_.height;    
                v4Pixel(0) = confirmedROV_.x;
                v4Pixel(1) = confirmedROV_.y + confirmedROV_.height;                    
                Vector3f v1Cam, v2Cam, v3Cam, v4Cam; // 相机坐标系下的4个顶点，顺时针，左上角到左下角依次为1/2/3/4
                float range = 1;
                v1Cam = utility_.project2dTo3d(v1Pixel, K_, range);
                v2Cam = utility_.project2dTo3d(v2Pixel, K_, range);
                v3Cam = utility_.project2dTo3d(v3Pixel, K_, range);
                v4Cam = utility_.project2dTo3d(v4Pixel, K_, range);
                Vector3f v1Cam2, v2Cam2, v3Cam2, v4Cam2; // 相机坐标系下的4个顶点，顺时针，左上角到左下角依次为1/2/3/4
                float range2 = 3;
                v1Cam2 = utility_.project2dTo3d(v1Pixel, K_, range2);
                v2Cam2 = utility_.project2dTo3d(v2Pixel, K_, range2);
                v3Cam2 = utility_.project2dTo3d(v3Pixel, K_, range2);
                v4Cam2 = utility_.project2dTo3d(v4Pixel, K_, range2);      

                //! 绘制梯形锥体--------------------------------------
                //! 梯形椎体，共有8个顶点，12条边，通过绘制12条线marker方式实现在rviz中的显示
                //! 顶点ID：从顶面开始到底面、面四边从左上角开始顺时针。1//2/3/4/5/6/7/8
                std::vector<geometry_msgs::Point> trapezoid3DLine;
                std::vector< std::vector<geometry_msgs::Point> > trapezoid3DAllLine;
                geometry_msgs::Point v1, v2, v3, v4, v5, v6, v7, v8; // ZED相机的点云坐标系为前左上坐标系
                // geometry_msgs
                // trapezoid3D_pub.publish();
                //! 顶面4条边
                v1.x = range;
                v1.y = -v1Cam(0);
                v1.z = -v1Cam(1);    
                v2.x = range;
                v2.y = -v2Cam(0);
                v2.z = -v2Cam(1);              
                trapezoid3DLine.push_back(v1);
                trapezoid3DLine.push_back(v2);
                trapezoid3DAllLine.push_back(trapezoid3DLine);         
                
                v3.x = range;
                v3.y = -v3Cam(0);
                v3.z = -v3Cam(1);            
                trapezoid3DLine.push_back(v2);
                trapezoid3DLine.push_back(v3);
                trapezoid3DAllLine.push_back(trapezoid3DLine); 

                v4.x = range;
                v4.y = -v4Cam(0);
                v4.z = -v4Cam(1);              
                trapezoid3DLine.push_back(v3);
                trapezoid3DLine.push_back(v4);
                trapezoid3DAllLine.push_back(trapezoid3DLine);      

                trapezoid3DLine.push_back(v4);
                trapezoid3DLine.push_back(v1);
                trapezoid3DAllLine.push_back(trapezoid3DLine);                
                //! 底面4条边
                v5.x = range2;
                v5.y = -v1Cam2(0);
                v5.z = -v1Cam2(1);    
                v6.x = range2;
                v6.y = -v2Cam2(0);
                v6.z = -v2Cam2(1);              
                trapezoid3DLine.push_back(v5);
                trapezoid3DLine.push_back(v6);
                trapezoid3DAllLine.push_back(trapezoid3DLine);         
                
                v7.x = range2;
                v7.y = -v3Cam2(0);
                v7.z = -v3Cam2(1);            
                trapezoid3DLine.push_back(v6);
                trapezoid3DLine.push_back(v7);
                trapezoid3DAllLine.push_back(trapezoid3DLine); 

                v8.x = range2;
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
                double width(0.02);                
                utility_.publish3DConvexHullMarkers("zed_left_camera_frame", color, width, trapezoid3DAllLine, trapezoid3D_pub);
            }
            //###################################################################################

            //! 在图像上绘制2D目标检测结果方框. http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage
            cv::circle(image_, center_, 10, CV_RGB(255,0,0), -1);
            // 绘制方框
            cv::rectangle(image_, roiP1_, roiP2_, CV_RGB(0,255,0), 3, 8, 0);
            // 绘制方框四条边的中点
            cv::circle(image_, pp1PixelFrame_, 8, CV_RGB(255,0,0), -1);
            cv::circle(image_, pp2PixelFrame_, 8, CV_RGB(0,255,0), -1);
            cv::circle(image_, pp3PixelFrame_, 8, CV_RGB(0,0,255), -1);
            cv::circle(image_, pp4PixelFrame_, 8, CV_RGB(255,255,0), -1);            
        }
        else // 检测失败
        {
            failedDetectCount++;
            cv::putText(image_, std::to_string(failedDetectCount), 
                cv::Point(200,300), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0,255,0), 10, 8, false);

            cv::putText(image_, "Failed Detect!", 
                cv::Point(100,100), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0,255,0), 3, 8, false);

            if(failedDetectCount <= failedDetectMaxCount){
                rovRect.x = confirmedROV_.x;
                rovRect.y = confirmedROV_.y;
                rovRect.width = confirmedROV_.width;
                rovRect.height = confirmedROV_.height;
            }

            if(failedDetectCount > failedDetectMaxCount){
                cv::putText(image_, "Failed Detect Max count reached, set rect to the whole image window!", 
                    cv::Point(100,150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2, 8, false);    

                rovRect.x = 10;
                rovRect.y = 10;
                rovRect.width = std::floor(rawImageW*0.8);
                rovRect.height = std::floor(rawImageH*0.8);                
            }

            rovRect_pub.publish(rovRect);
        }

        // 发布图像
        imgPub_.publish(br->toImageMsg());            
        // ================================= END =================================================

        /* 
        // 卡尔曼滤波，滤波后的2D目标box中心：蓝色
        // predict
        if(debug_) cout<<"Prediction ..."<<endl;
        kf.Predict();
        if(debug_) cout<<"predicted x_ = "<<endl<<kf.x_<<endl;
        
        // measurement update
        if(debug_) cout<<"Update ..."<<endl;
        kf.Update(measurements);        
        if(debug_){
            cout << "fused x_= " << endl<<kf.x_ << endl;
            cout << "P_= " << endl<<kf.P_ << endl;  
        } 

        // 绘制融合后的目标中心：蓝色
        cv::Point2d box2dCenterFused;
        box2dCenterFused.x = kf.x_(0);
        box2dCenterFused.y = kf.x_(1);
        cv::circle(image_, box2dCenterFused, 10, CV_RGB(0,0,255), -1);
        */
    }


    /* 
    void CascadeDetection::imgCallback(  const sensor_msgs::ImageConstPtr& imgMsg){
        cout<<endl<<endl;
        cout<<"IMAGE SUBSCRIBE MODE:  image ONLY callback"<<endl;

        //! 获取RGB图像，转成opencv格式，并处理后发布出去
        // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
        // cv::Mat image_;
        cv_bridge::CvImagePtr br;
        try{
            br = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
            image_ = br->image;
            // cout<<"input image size: "<<image_.cols<<"  "<<image_.rows<<endl;
        }catch(cv_bridge::Exception& ex){
            ROS_ERROR("Failed to convert image!");
            return;
        }

        //! 开始检测
        confirmedObjectFlag_ = false;
        cout<<"Detection begin ...."<<endl;
        confirmedROV_ = detect(image_, cascade_, nestedCascade_, scale_);
        cout<<"Done."<<endl;

        //! 将目标检测结果（2D BBX）发布出去，将被3D点云detector节点订阅
        jsk_recognition_msgs::Rect rovRect;
        rovRect.x = confirmedROV_.x;
        rovRect.y = confirmedROV_.y;
        rovRect.width = confirmedROV_.width;
        rovRect.height = confirmedROV_.height;
        // 只有当成功检测出目标才把目标rect发布出去，这样就方便订阅了该话题的节点不用去判断该目标rect是否有效了（长宽高不为0）
        if(confirmedObjectFlag_){
            rovRect_pub.publish(rovRect);

        }


        // 绘制2D目标box中心：红色
        cv::Point2d box2dCenter;
        box2dCenter.x = confirmedROV_.x + 0.5*confirmedROV_.width;
        box2dCenter.y = confirmedROV_.y + 0.5*confirmedROV_.height;
        cv::circle(image_, box2dCenter, 10, CV_RGB(255,0,0), -1);

        //! 在图像上绘制3D点云目标检测结果投影到图像平面后的方框. 
        bool project3Dto2DFlag = false;
        if(project3Dto2DFlag){
            // http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage
            cv::circle(image_, center_, 10, CV_RGB(255,0,0), -1);
            // 绘制方框
            cv::rectangle(image_, roiP1_, roiP2_, CV_RGB(0,255,0), 3, 8, 0);
            // 绘制方框四条边的中点
            cv::circle(image_, pp1PixelFrame_, 8, CV_RGB(255,0,0), -1);
            cv::circle(image_, pp2PixelFrame_, 8, CV_RGB(0,255,0), -1);
            cv::circle(image_, pp3PixelFrame_, 8, CV_RGB(0,0,255), -1);
            cv::circle(image_, pp4PixelFrame_, 8, CV_RGB(255,255,0), -1);
        }

        // 发布图像，用于显示
        imgPub_.publish(br->toImageMsg());        
    }
    */


    /*
    暂停使用
    */
   /*
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
    */

}


