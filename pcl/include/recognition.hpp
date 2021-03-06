#ifndef RECOGNITION_H
#define RECOGNITION_H

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace PCL
{
class Recognition
{
public:
    struct Model
    {
        PointCloud::Ptr model;
        PointCloud::Ptr keyPoints;
        pcl::PointCloud<pcl::Normal>::Ptr normals;
        pcl::PointCloud<pcl::SHOT352>::Ptr descriptors;
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr refeFrame;
        std::string fileName;
        
    } model_;

    struct Parameters
    {
        //! normals.
        unsigned int setKSearch;

        //! uniform sample.
        float setRadiusSearch;

        //! descriptors.
        float setRadiusSearch_descriptors;

        //! reference frame.
        float setRadiusSearch_rf;

        //! Hough3DGrouping.
        unsigned int setHoughBinSize;
        float setHoughThreshold;

    } param_;

public:
    Recognition();
    ~Recognition();

    void computeNormals(PointCloud::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals);

    void uniformSample(PointCloud::Ptr& cloud, PointCloud::Ptr& outCloud);

    void computeDescriptors(PointCloud::Ptr& cloud, 
                            PointCloud::Ptr& keyPoints,
                            pcl::PointCloud<pcl::Normal>::Ptr& normals,
                            pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors);    

    void findCorrespondences(pcl::PointCloud<pcl::SHOT352>::Ptr& modelDescriptors,
                            pcl::PointCloud<pcl::SHOT352>::Ptr& sceneDescriptors,
                            pcl::CorrespondencesPtr& correspondences);

    void computeReferenceFrame(PointCloud::Ptr& cloud,
                            PointCloud::Ptr& keyPoints,
                            pcl::PointCloud<pcl::Normal>::Ptr& normals,
                            pcl::PointCloud<pcl::ReferenceFrame>::Ptr& rf);    

    void Hough3DGrouping(PointCloud::Ptr& modelKeyPoints,
                        PointCloud::Ptr& sceneKeyPoints,
                        pcl::PointCloud<pcl::ReferenceFrame>::Ptr& modelRF,
                        pcl::PointCloud<pcl::ReferenceFrame>::Ptr& sceneRF);

    // void GeometricConsistencyGrouping();
}; // class.
} // namespace.
#endif
