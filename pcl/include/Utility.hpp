#ifndef UTILITY_H
#define UTILITY_H

// system information
#include <unistd.h>
#include <pwd.h>

//! Eigen
#include <Eigen/Dense>

//! ROS
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>


namespace bluerov_detection_tracking
{
class Utility
{
public:
    inline std::string getUserName(){
        struct passwd* pwd;
        uid_t userid;
        userid = getuid();
        pwd = getpwuid(userid);
        // cout<<pwd->pw_name<<endl;
        return pwd->pw_name;
    }

    /*
    # Intrinsic camera matrix for the raw (distorted) images.
    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]
    # Projects 3D points in the camera coordinate frame to 2D pixel
    # coordinates using the focal lengths (fx, fy) and principal point
    # (cx, cy).
    float64[9]  K # 3x3 row-major matrix
    */
    inline Eigen::Vector3f project3dTo2d(Eigen::Vector3f p3, Eigen::Matrix3f K){
        // 2dpoint = K*3dpoint
        Eigen::Vector3f p2;
        p2 = K*p3;
        // p2(0) = p2(0) - p3(2)*K(0,2);
        // p2(1) = p2(1) - p3(2)*K(1,2);
        p2(0) = p2(0)/(p3(2)-1);
        p2(1) = p2(1)/(p3(2)-1);

        return p2;
    }

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
    inline Eigen::Vector3f project3dTo2d(Eigen::Vector3f p3, Eigen::MatrixXf P){
        Eigen::Vector4f p4;
        p4(0) = p3(0);
        p4(1) = p3(1);
        p4(2) = p3(2);
        p4(3) = 1;

        Eigen::Vector3f p2;
        p2 = P*p4;

        p2(0) = p2(0)/p2(2);
        p2(1) = p2(1)/p2(2);
        // p2(0) = p2(0)/(p2(2)-0.9);
        // p2(1) = p2(1)/(p2(2)-0.9);

        return p2;
    }    


    /*
        说明：此方法存在计算误差，暂时停用！
        图像坐标投影到相机坐标，通过深度值计算，适合RGBD相机
        输入：图像坐标
             3*3内参矩阵
             深度归一化scale
             归一化后的深度值
        输出：三维空间坐标

        依据小孔投影模型原理：
        # u = fx*x/z + cx;
        # v = fy*y/z + cy;
    */
    inline Eigen::Vector3f project2dTo3d(Eigen::Vector2f p2, Eigen::Matrix3f k, float depthScale, float depth){
       Eigen::Vector3f p3;
       p3(0) = (p2(0) - k(0,2))/k(0,0);
       p3(1) = (p2(1) - k(1,2))/k(1,1);
       p3(2) = depth/depthScale;

       return p3;
   }


    /*
        说明：只要知道准确的相机内参和准确的点距离，那么此方法计算无误差，已经经过验证了
        图像坐标投影到相机坐标，通过实际距离计算（没有归一化尺度）
        输入：图像坐标
             3*3内参矩阵
             空间点在相机坐标系下到相机坐标原点的距离（实际深度值，非归一化深度值！）
        输出：三维空间坐标

        原理：
        p_camera = p_pixel * range * K_inverse
    */
    inline Eigen::Vector3f project2dTo3d(Eigen::Vector2f p2, Eigen::Matrix3f k, float range){
       Eigen::Vector3f p3;
       Eigen::Vector3f p2_normalized;
       p2_normalized(0) = p2(0);
       p2_normalized(1) = p2(1);
       p2_normalized(2) = 1.0;

       p3 = k.inverse() * range * p2_normalized;

       return p3;
   }


    /*
        发布两个坐标系之间的变换关系TF
        输入：父坐标系名称
             子坐标系名称
             变换位姿
        输出：无
    */
    void sendTF(std::string fatherFrame, 
                std::string childFrame,
                geometry_msgs::PoseStamped pose);


    /*
        目标位置转换到NED坐标系
        输入：当前位置
             目标初始化位置
        输出：NED位置坐标
    */
    void toNED( geometry_msgs::PoseStamped poseIn,
                Eigen::Vector3f initialPose,
                geometry_msgs::PoseStamped& poseOut); // 输出必须以引用传递，函数内部修改变量值才能保存到全局变量中！



    /*
        目标位置转换到locked_pose坐标系
        输入：当前位置
             目标初始化位置
        输出：locked_pose位置坐标
    */
    void toLockedPoseFrame( geometry_msgs::PoseStamped poseIn,
                            Eigen::Vector3f initialPose,
                            geometry_msgs::PoseStamped& poseOut); // 输出必须以引用传递，函数内部修改变量值才能保存到全局变量中！



    // void publish3DConvexHullMarkers(   std::string frame,
    //                                             std::vector<geometry_msgs::Point> vertex,
    //                                             ros::Publisher publisher);


    /*
        发布三维凸包markers到rviz显示
        输入：markers的坐标系
             颜色
             线条粗细
             保存了构成三维凸包的所有边的vector
             ROS发布器
        输出：
    */
    void publish3DConvexHullMarkers(std::string frame,
                                    std_msgs::ColorRGBA color,
                                    double width,    
                                    std::vector<std::vector<geometry_msgs::Point>> vertex,
                                    ros::Publisher publisher);



    /*
        获取球型marker
        输入：marker所属坐标系
             marker颜色、透明度、尺寸
             marker位置
        输出：球型marker
    */
    void sphereMarker(  std_msgs::Header &header,
                        Eigen::VectorXf &CAS,
                        Eigen::VectorXd position,
                        visualization_msgs::Marker &marker);


    // void Utility::sphereMarkerArray(bool &isCenter,
    //                                 std_msgs::Header &header, 
    //                                 VectorXd &c_a_s, 
    //                                 std::vector<DetectedObject> &objList,
    //                                 visualization_msgs::MarkerArray &markerArray);



    /*
        发布轨迹到RVIZ中显示
        输入：轨迹所属坐标系
             轨迹颜色
             轨迹点
             轨迹发布器
        输出：轨迹，类型为visualization_msgs::Marker
    */
    void publishPath(   std::string frameId, 
                        std_msgs::ColorRGBA colorRGBA,
                        geometry_msgs::Point point,
                        ros::Publisher publisher,
                        visualization_msgs::Marker& path); // path必须以引用方式传入，在压入位置点时才能保存到全局path变量中！



};

}
#endif