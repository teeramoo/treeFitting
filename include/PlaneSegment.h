//
// Created by teeramoo on 23/4/2561.
//
#ifndef TREEFIT_PLANESEGMENT_H
#define TREEFIT_PLANESEGMENT_H


#include <boost/thread/thread.hpp>


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


class PlaneSegment {

public:

    PlaneSegment();
    PlaneSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr, bool &bSetOptimization);

    ~PlaneSegment();

    bool performSegmentation();

    // ** Implement all of these

    void setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_point_cloud_ptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getInputPointCloud();

    //

    void setPlaneCoefficient(pcl::ModelCoefficients::Ptr &_planeCoefficients);

    pcl::ModelCoefficients::Ptr getPlaneCoefficient();

    //

    void setPlaneInliers(pcl::PointIndices::Ptr &_planeInliers);

    pcl::PointIndices::Ptr getPlaneInliers();

    //

    void setPlanePointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_planePointcloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPlanePointCloud_ptr();

    //

    void setNoPlanePointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_noPlanePointcloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getNoPlanePointCloud_ptr();

    //

    void calculatePlaneVector();

    void setPlaneVector(Eigen::Vector3f &_planeVector);

    Eigen::Vector3f getPlaneVector();

private:

    pcl::SACSegmentation<pcl::PointXYZRGB> planeSegmentator;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud;
    pcl::ModelCoefficients::Ptr planeCoefficients;
    pcl::PointIndices::Ptr planeInliers;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePointcloud_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr noPlanePointcloud_ptr;
    Eigen::Vector3f planeVector;

};

#endif //TREEFIT_PLANESEGMENT_H
