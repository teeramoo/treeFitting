//
// Created by teeramoo on 24/4/2561.
//

#ifndef TREEFIT_NORMALCALCULATION_H
#define TREEFIT_NORMALCALCULATION_H
//Boost libraries
#include <boost/thread/thread.hpp>

// PCL libraries

#include <pcl/common/common_headers.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class NormalEstimator {

public:
    NormalEstimator();
    NormalEstimator(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointClod, double searchRadius);
    NormalEstimator(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointClod, int &Ksearch);

    ~NormalEstimator();

    void calculate();

    void setEstimator(pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> _normalEstimator);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> getEstimator();

    void setCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr &_cloud_normals);
    pcl::PointCloud<pcl::Normal>::Ptr getCloudNormal();

    void setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_inputPointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getInputPointCloud();

private:
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud;
};


#endif //TREEFIT_NORMALCALCULATION_H
