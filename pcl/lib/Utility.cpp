#include "Utility.hpp"


namespace bluerov_detection_tracking
{

    void Utility::sendTF(   std::string fatherFrame, 
                            std::string childFrame,
                            geometry_msgs::PoseStamped pose){
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped tf;

        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = fatherFrame;
        tf.child_frame_id = childFrame;

        tf.transform.translation.x = pose.pose.position.x;
        tf.transform.translation.y = pose.pose.position.y;
        tf.transform.translation.z = pose.pose.position.z;

        tf.transform.rotation.x = pose.pose.orientation.x;
        tf.transform.rotation.y = pose.pose.orientation.y;
        tf.transform.rotation.z = pose.pose.orientation.z;
        tf.transform.rotation.w = pose.pose.orientation.w;

        br.sendTransform(tf);
    }



    void Utility::toNED(geometry_msgs::PoseStamped poseIn,
                        Eigen::Vector3f initialPose,
                        geometry_msgs::PoseStamped& poseOut)
    {
        poseOut.header = poseIn.header;
        
        poseOut.pose.position.x = poseIn.pose.position.x - initialPose(0); // base_link 2 NED
        poseOut.pose.position.y = -1 * (poseIn.pose.position.y - initialPose(1)); 
        poseOut.pose.position.z = -1 * (poseIn.pose.position.z - initialPose(2));   
 

      //! online查询 方式
      // tf2_ros::Buffer buf;
      // tf2_ros::TransformListener listener(buf);
      // geometry_msgs::TransformStamped trans;              
      // trans = buf.lookupTransform("bluerov2h_follower/base_link", 
      //                             "bluerov2h_follower/cloud_link", 
      //                             ros::Time(0), ros::Duration(1.0)); // 太耗时啦，改成提前计算出变换矩阵

      //! offline 方式
      // trans.transform.translation.x = 0.2; // base_link
      // trans.transform.translation.y = 0.0;
      // trans.transform.translation.z = -0.2;
      // trans.transform.rotation.x = -0.5;
      // trans.transform.rotation.y = 0.5;
      // trans.transform.rotation.z = -0.5;
      // trans.transform.rotation.w = 0.5;       
      // tf2::doTransform(localizationResult.pose.pose.position, localizationResult.poseNED.pose.position, trans);          
   }    



    void Utility::toLockedPoseFrame(geometry_msgs::PoseStamped poseIn,
                                    Eigen::Vector3f initialPose,
                                    geometry_msgs::PoseStamped& poseOut)
    {
        poseOut.header = poseIn.header;
        
        poseOut.pose.position.x = poseIn.pose.position.x - initialPose(0); 
        poseOut.pose.position.y = poseIn.pose.position.y - initialPose(1); 
        poseOut.pose.position.z = poseIn.pose.position.z - initialPose(2);   
    }



    void Utility::publish3DConvexHullMarkers(   std::string frame,
                                                std_msgs::ColorRGBA color,
                                                double width,
                                                std::vector< std::vector<geometry_msgs::Point> > vertex,
                                                ros::Publisher publisher){
        visualization_msgs::Marker line;
        line.header.frame_id = frame;
        line.action = visualization_msgs::Marker::ADD;

        line.pose.orientation.w = 1.0;
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.scale.x = width; //线条粗细
        line.color = color;

        for(std::vector< std::vector<geometry_msgs::Point> >::iterator it = vertex.begin(); it!=vertex.end(); ++it){
            for(std::vector<geometry_msgs::Point>::iterator it2 = it->begin(); it2!=it->end(); ++it2){
                line.points.push_back(*it2);
            }
        }

        publisher.publish(line);
    }


    void Utility::sphereMarker( std_msgs::Header &header,
                                Eigen::VectorXf &CAS,
                                Eigen::VectorXd position,
                                visualization_msgs::Marker &marker){
      marker.type = visualization_msgs::Marker::SPHERE;   
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(0);
      marker.header = header; 
      marker.color.r = CAS[0];
      marker.color.g = CAS[1];
      marker.color.b = CAS[2];          
      marker.color.a = CAS[3];
      marker.scale.x = marker.scale.y = marker.scale.z = CAS[4];
      marker.pose.orientation.w = 1.0;    

      marker.pose.position.x = position[0]; 
      marker.pose.position.y = position[1]; 
      marker.pose.position.z = position[2];             
    }    


    void Utility::publishPath(  std::string frameId, 
                                std_msgs::ColorRGBA colorRGBA,
                                geometry_msgs::Point point,
                                ros::Publisher publisher,
                                visualization_msgs::Marker& path)
    {
      // http://wiki.ros.org/rviz/Tutorials/Markers:%20Points%20and%20Lines
      // http://wiki.ros.org/rviz/DisplayTypes/Marker#Line_Strip_.28LINE_STRIP.3D4.29
      // http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html

      path.header.frame_id = frameId;
      path.header.stamp = ros::Time::now();
      path.action = visualization_msgs::Marker::ADD;
      path.pose.orientation.w = 1.0;
      path.id = 0;
      path.type = visualization_msgs::Marker::LINE_STRIP;
      path.scale.x = 0.001;
      path.color = colorRGBA;    

      path.points.push_back(point);

      publisher.publish(path);
    }

    // void Utility::sphereMarkerArray(bool &isCenter,
    //                                 std_msgs::Header &header, 
    //                                 VectorXd &CAS, 
    //                                 std::vector<DetectedObject> &objList,
    //                                 visualization_msgs::MarkerArray &markerArray){
    //   // visualization_msgs::MarkerArray markerArray;  
    //   visualization_msgs::Marker marker;

    //   marker.type = visualization_msgs::Marker::SPHERE;   
    //   marker.action = visualization_msgs::Marker::ADD;
    //   marker.lifetime = ros::Duration(0);
    //   marker.header = header; 
    //   marker.color.r = CAS[0];
    //   marker.color.g = CAS[1];
    //   marker.color.b = CAS[2];          
    //   marker.color.a = CAS[3];
    //   marker.scale.x = marker.scale.y = marker.scale.z = CAS[4];
    //   marker.pose.orientation.w = 1.0;     

    //   int markerID = 0;
    //   markerArray.markers.clear();
    //   for(size_t i=0; i<objList.size(); i++) 
    //   {
    //       marker.id = markerID;

    //       // 点云坐标系(x,y,z)和本体坐标系(z,x,y的不同！此处，发布的为点云坐标系下。
    //       if(isCenter){
    //         marker.pose.position.x = objList[i].center.x(); // in point cloud(camera) frame.
    //         marker.pose.position.y = objList[i].center.y(); 
    //         marker.pose.position.z = objList[i].center.z();    
    //         markerArray.markers.push_back(marker);     
    //         marker.pose.position.x = objList[i].center.z();  // in base_link frame.
    //         marker.pose.position.y = -objList[i].center.x(); 
    //         marker.pose.position.z = -objList[i].center.y();  
    //       }else{
    //         marker.pose.position.x = objList[i].centroid.x; // in point cloud(camera) frame.
    //         marker.pose.position.y = objList[i].centroid.y; 
    //         marker.pose.position.z = objList[i].centroid.z;    
    //         markerArray.markers.push_back(marker);
    //         marker.pose.position.x = objList[i].centroid.z;  // in base_link frame.
    //         marker.pose.position.y = -objList[i].centroid.x; 
    //         marker.pose.position.z = -objList[i].centroid.y;   
    //       }
    //       ++markerID;  
    //   }  
    // }


}