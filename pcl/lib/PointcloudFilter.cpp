#include "PointcloudFilter.hpp"

using namespace std;

namespace bluerov_detection_tracking
{
    PointCloudFilter::PointCloudFilter(){
        // convexHullPoints_pub = node_.advertise<sensor_msgs::PointCloud2>("convex_hull_points", 20);
    };
    PointCloudFilter::~PointCloudFilter(){};

    bool PointCloudFilter::Filter(const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloudFiltered)
    {
        if(cloud->size()==0){
            ROS_ERROR("Input cloud is 0.");
            return false;
        }

        if(cloudFiltered==NULL){
            ROS_ERROR("Output is NULL.");
            return false;
        }
        
        *cloudFiltered = *cloud;

        //! passthrough filter.
        if(passthroughFilter_){
            ros::Time t0 = ros::Time::now();
            pcl::PassThrough<pcl::PointXYZ> pass_filter;  

            pass_filter.setInputCloud(cloudFiltered);

            pass_filter.setFilterFieldName("x");
            pass_filter.setFilterLimits(setFilterLimits_X1_, setFilterLimits_X2_); // ！根据真实场景调整参数
            pass_filter.filter(*cloudFiltered);     

            pass_filter.setFilterFieldName("y");
            pass_filter.setFilterLimits(setFilterLimits_Y1_, setFilterLimits_Y2_); // ！根据真实场景调整参数
            pass_filter.filter(*cloudFiltered);      

            pass_filter.setFilterFieldName("z");
            pass_filter.setFilterLimits(setFilterLimits_Z1_, setFilterLimits_Z2_); // ！根据真实场景调整参数
            pass_filter.filter(*cloudFiltered);

            if(debug_) cout<<"Passthrough filtered cloud size = "<<cloudFiltered->size()<<"\t"
                <<"Time used = "<<ros::Time::now()-t0<<endl;

            if(cloudFiltered->size()==0) return false;
        }

        //! voxel grid downsample.
        if(voxelDownsample_){
            ros::Time t0 = ros::Time::now();

            pcl::VoxelGrid<pcl::PointXYZ> voxcelFilter;
            voxcelFilter.setInputCloud(cloudFiltered);
            voxcelFilter.setLeafSize(setLeafSize_, setLeafSize_, setLeafSize_); // ！根据真实场景调整参数
            voxcelFilter.filter(*cloudFiltered);     

            if(debug_) cout<<"Downsampled cloud size = "<<cloudFiltered->size()<<"\t"
                <<"Time used = "<<ros::Time::now()-t0<<endl;                              
        } 
        
        //! remove ground.
        if(removeGround_ && cloudFiltered->size()!=0){
            ros::Time t0 = ros::Time::now();

            // extract gound
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients);
            seg.setOptimizeCoefficients (true);
            seg.setModelType ( pcl::SACMODEL_PLANE );
            seg.setMethodType ( pcl::SAC_RANSAC );
            seg.setDistanceThreshold ( 0.01 );
            seg.setInputCloud ( cloudFiltered );
            seg.segment ( *inliers, *coef );

            // remove ground
            pcl::ExtractIndices<pcl::PointXYZ> ex;
            ex.setInputCloud(cloudFiltered);
            ex.setIndices(inliers);
            ex.setNegative(true);
            ex.filter(*cloudFiltered);    

            if(debug_) cout<<"Removed ground cloud size = "<<cloudFiltered->size()<<"\t"
                <<"Time used = "<<ros::Time::now()-t0<<endl;                       
        }

        if(outlierRemove_){
            ros::Time t0 = ros::Time::now();
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            outrem.setInputCloud(cloudFiltered);
            outrem.setRadiusSearch(setRadiusSearch_);
            outrem.setMinNeighborsInRadius(setMinNeighborsInRadius_);
            outrem.filter(*cloudFiltered);
            if(debug_) cout<<"RadiusOutlierRemoval cloud size = "<<cloudFiltered->size()<<"\t"
                <<"Time used = "<<ros::Time::now()-t0<<endl;  

            ros::Time t1 = ros::Time::now();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloudFiltered);
            sor.setMeanK(setMeanK_);
            sor.setStddevMulThresh(setStddevMulThresh_);
            sor.filter(*cloudFiltered);
            if(debug_) cout<<"StatisticalOutlierRemoval cloud size = "<<cloudFiltered->size()<<"\t"
                <<"Time used = "<<ros::Time::now()-t1<<endl;              
        }

        if(cloudFiltered->size()==0){
            ROS_ERROR("Cloud filtered size is 0.");
            return false;
        }        

        return true;
    }


    bool PointCloudFilter::convexHullFilter(const PointCloud::ConstPtr& points, 
                                            std::vector<Eigen::Vector3f> vertex,
                                            PointCloud::Ptr pointsFiltered)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertexPtr(new pcl::PointCloud<pcl::PointXYZ>);

        // 三维凸包顶点
        int i = 0;
        for(std::vector<Eigen::Vector3f>::iterator it = vertex.begin(); it!=vertex.end(); ++it){
            Eigen::Vector3f vertexTem;
            vertexTem = *it; // 相机坐标系（z轴朝前）

            // 转换到点云坐标系（x轴朝前）
            Eigen::Vector3f vertexCloud;
            vertexCloud(0) = vertexTem(2);
            vertexCloud(1) = -vertexTem(0);
            vertexCloud(2) = -vertexTem(1);

            vertexPtr->push_back(pcl::PointXYZ(vertexCloud(0), vertexCloud(1), vertexCloud(2)));
            // vertexPtr->push_back(pcl::PointXYZ(vertexTem(0), vertexTem(1), vertexTem(2)));

            if(false){
                cout<<"vertex "<<i<<" = "<<vertexTem<<endl;
                ++i;
            }

            // vertexPtr->push_back(pcl::PointXYZ(*it.at(0), *it(1), *it(2)));
        }

        // 构造三维凸包，需要用到concave_hull类
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(vertexPtr);
        std::vector<pcl::Vertices> polygons;
        pcl::PointCloud<pcl::PointXYZ>::Ptr surfaceHullPtr(new pcl::PointCloud<pcl::PointXYZ>); // 描述凸包形状
        hull.reconstruct(*surfaceHullPtr, polygons);

        // 创建CropHull滤波器
        // PointCloud::Ptr filteredCloud;
        pcl::CropHull<pcl::PointXYZ> filter;
        filter.setDim(3);
        filter.setInputCloud(points);
        filter.setHullIndices(polygons); // 封闭多边形的顶点
        filter.setHullCloud(surfaceHullPtr); // 封闭多边形的形状
        filter.filter(*pointsFiltered);

        return true;
    }               

} // namespace