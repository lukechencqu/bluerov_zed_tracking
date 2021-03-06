  visualization_msgs::MarkerArray boundingBoxCentroidList;  
  for(size_t i=0; i<objList.size(); i++) 
  {
    boundingBoxCentroidList[i].type = visualization_msgs::Marker::SPHERE;   
    boundingBoxCentroidList[i].id = 0;
    boundingBoxCentroidList[i].action = visualization_msgs::Marker::ADD;
    boundingBoxCentroidList[i].lifetime = ros::Duration(0);
    boundingBoxCentroidList[i].header = boundingBoxArray.header; 
    // boundingBoxCentroidList.header.frame_id = cloud->header.frame_id; //"/map";
    // boundingBoxCentroidList.header.stamp = cloud->header.stamp; //ros::Time::now();    
    boundingBoxCentroidList[i].scale.x = boundingBoxCentroidList.scale.y = boundingBoxCentroidList.scale.z = 0.05;;
    boundingBoxCentroidList[i].color.g = 1.0;
    boundingBoxCentroidList[i].color.a = 1.0;
    boundingBoxCentroidList[i].pose.orientation.w = 1.0;    
    boundingBoxCentroidList[i].pose.position.x = objList[i].centroid_.x; 
    boundingBoxCentroidList[i].pose.position.y = objList[i].centroid_.y; 
    boundingBoxCentroidList[i].pose.position.z = objList[i].centroid_.z;     
  }  
  boundingBoxCentroidMarker_pub.publish(boundingBoxCentroidList);  
  /*
  visualization_msgs::Marker boundingBoxCentroidList;
  boundingBoxCentroidList.type = visualization_msgs::Marker::SPHERE_LIST;   
  boundingBoxCentroidList.id = 0;
  boundingBoxCentroidList.action = visualization_msgs::Marker::ADD;
  boundingBoxCentroidList.lifetime = ros::Duration(0);
  boundingBoxCentroidList.header = boundingBoxArray.header; 
  // boundingBoxCentroidList.header.frame_id = cloud->header.frame_id; //"/map";
  // boundingBoxCentroidList.header.stamp = cloud->header.stamp; //ros::Time::now();    
  boundingBoxCentroidList.scale.x = boundingBoxCentroidList.scale.y = boundingBoxCentroidList.scale.z = 0.05;;
  // boundingBoxCentroidList.color.g = 1.0;
  // boundingBoxCentroidList.color.a = 1.0;
  boundingBoxCentroidList.pose.orientation.w = 1.0;    

  std_msgs::ColorRGBA color;
  color.g = 1.0;
  color.a = 1.0;
  for(size_t i=0; i<objList.size(); i++) 
  {
    geometry_msgs::Point p;
    p.x = objList[i].centroid_.x;
    p.y = objList[i].centroid_.y;
    p.z = objList[i].centroid_.z;
    boundingBoxCentroidList.points.push_back(p); 
    boundingBoxCentroidList.colors.push_back(color); 
  }
  boundingBoxCentroidMarker_pub.publish(boundingBoxCentroidList);
  
  */

