#ifndef POINT_CLOUD_FILTER_H
#define POINT_CLOUD_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>


namespace bluerov_detection_tracking
{

class PointCloudFilter
{
public:

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    PointCloudFilter();
    ~PointCloudFilter();

    bool Filter(const PointCloud::ConstPtr& points, PointCloud::Ptr pointsFiltered);

    bool convexHullFilter(  const PointCloud::ConstPtr& points, 
                            std::vector<Eigen::Vector3f> vertex,
                            PointCloud::Ptr pointsFiltered);

public:

    bool passthroughFilter_;
    bool removeGround_;
    bool voxelDownsample_;
    bool outlierRemove_;

    //! passthrough filter.
    float setFilterLimits_X1_, setFilterLimits_X2_;
    float setFilterLimits_Y1_, setFilterLimits_Y2_;
    float setFilterLimits_Z1_, setFilterLimits_Z2_;

    //! downsmaple.
    float setLeafSize_;

    //! radius outlier remove.
    float setRadiusSearch_;
    float setMinNeighborsInRadius_;

    //! statistical outlier remove.
    float setMeanK_;
    float setStddevMulThresh_;

    bool debug_;        


private:

    // ros::NodeHandle node_;

    // // 发布器
    // ros::Publisher convexHullPoints_pub;

}; // class

} // namespace

#endif