#include "recognition.hpp"

using namespace std;

namespace PCL
{
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> TM;
    vector<pcl::Correspondences> clusteredCorrespondences;

    Recognition::Recognition(){
        model_.model.reset(new PointCloud);
        model_.keyPoints.reset(new PointCloud);
        model_.normals.reset(new pcl::PointCloud<pcl::Normal>);
        model_.refeFrame.reset(new pcl::PointCloud<pcl::ReferenceFrame>);     
    };

    Recognition::~Recognition(){};

    void Recognition::computeNormals(PointCloud::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals){
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEst;
        normalEst.setKSearch(param_.setKSearch);
        normalEst.setInputCloud(cloud);
        normalEst.compute(*normals);
    }

    void Recognition::uniformSample(PointCloud::Ptr& cloud, PointCloud::Ptr& outCloud){
        pcl::UniformSampling<pcl::PointXYZ> uniSample;
        uniSample.setInputCloud(cloud);
        uniSample.setRadiusSearch(param_.setRadiusSearch);
        uniSample.filter(*outCloud);
    }

    void Recognition::computeDescriptors(PointCloud::Ptr& cloud, 
                                        PointCloud::Ptr& keyPoints,
                                        pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                        pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors){
        pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descrEst;
        descrEst.setRadiusSearch(param_.setRadiusSearch_descriptors);
        descrEst.setInputCloud(keyPoints);
        descrEst.setInputNormals(normals);
        descrEst.setSearchSurface(cloud);
        descrEst.compute(*descriptors);
    }

    void Recognition::findCorrespondences(pcl::PointCloud<pcl::SHOT352>::Ptr& modelDescriptors,
                                        pcl::PointCloud<pcl::SHOT352>::Ptr& sceneDescriptors,
                                        pcl::CorrespondencesPtr& correspondences){
        // pcl::CorrespondencesPtr correspondences (new pcl::Correspondences());
        pcl::KdTreeFLANN<pcl::SHOT352> matchSearch;

        matchSearch.setInputCloud(modelDescriptors);

        for(size_t i=0; i<sceneDescriptors->size(); ++i){
            vector<int> neighIndices(1);
            vector<float> neighSqrDists(1);
            if(!isfinite(sceneDescriptors->at(i).descriptor[0])) continue;

            int foundNeighs = matchSearch.nearestKSearch(sceneDescriptors->at(i), 1, neighIndices, neighSqrDists);
            if(foundNeighs==1 && neighSqrDists[0]<0.25f){
                pcl::Correspondence corr(neighIndices[0], static_cast<int>(i), neighSqrDists[0]);
                correspondences->push_back(corr);
            }
        }
    }

    void Recognition::computeReferenceFrame(PointCloud::Ptr& cloud,
                                            PointCloud::Ptr& keyPoints,
                                            pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                            pcl::PointCloud<pcl::ReferenceFrame>::Ptr& rf){
        // pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf(new pcl::PointCloud<pcl::ReferenceFrame);
        pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rfEst;
        rfEst.setFindHoles(true);
        rfEst.setRadiusSearch(param_.setRadiusSearch_rf);
        rfEst.setInputCloud(keyPoints);
        rfEst.setInputNormals(normals);
        rfEst.setSearchSurface(cloud);
        rfEst.compute(*rf);
    }

    void Recognition::Hough3DGrouping(PointCloud::Ptr& modelKeyPoints,
                                    PointCloud::Ptr& sceneKeyPoints,
                                    pcl::PointCloud<pcl::ReferenceFrame>::Ptr& modelRF,
                                    pcl::PointCloud<pcl::ReferenceFrame>::Ptr& sceneRF){
        pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> cluster;
        cluster.setHoughBinSize(param_.setHoughBinSize);
        cluster.setHoughThreshold(param_.setHoughThreshold);
        cluster.setUseInterpolation(true);
        cluster.setUseDistanceWeight(false);

        cluster.setInputCloud(modelKeyPoints);
        cluster.setInputRf(modelRF);
        cluster.setSceneCloud(sceneKeyPoints);
        cluster.setSceneRf(sceneRF);

        cluster.recognize(TM, clusteredCorrespondences);
    }
}