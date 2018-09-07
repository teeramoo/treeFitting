//
// Created by teeramoo on 24/4/2561.
//

#include "NormalEstimator.h"

NormalEstimator::NormalEstimator(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud, double searchRadius) {

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> _ne;
    _ne.setInputCloud (inputPointCloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    _ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    _ne.setRadiusSearch (searchRadius);
//    _ne.setKSearch(10);
    setEstimator(_ne);
    setInputPointCloud(inputPointCloud);

}

NormalEstimator::~NormalEstimator() {
 //   cout << "Normal calculator has been  destroyed" << endl;
}

void NormalEstimator::setEstimator(pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> _normalEstimator) {
    ne = _normalEstimator;
}

pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> NormalEstimator::getEstimator() {
    return ne;
}

void NormalEstimator::setCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr &_cloud_normals) {
    cloud_normals = _cloud_normals;
}

pcl::PointCloud<pcl::Normal>::Ptr NormalEstimator::getCloudNormal() {
    return cloud_normals;
}

void NormalEstimator::setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_inputPointCloud) {
    inputPointCloud = _inputPointCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr NormalEstimator::getInputPointCloud() {
    return inputPointCloud;
}

void NormalEstimator::calculate() {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);
    setCloudNormal(cloud_normals);
}