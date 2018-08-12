//
// Created by teeramoo on 23/4/2561.
//

#include "PlaneProcessor.h"

PlaneProcessor::~PlaneProcessor() {
 //   cout << "PlaneProcessor has been destroyed." << endl;
}


bool PlaneProcessor::segment() {

//    cout << "about to segment a plane" << endl;
    planeSegmenter.segment(*planeInliers, *planeCoefficients);
//    cout << "finish segmentation" << endl;

    if (planeInliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model from the given dataset.");

        return false;
    }

    std::cerr << "Plane model coefficients: " << planeCoefficients->values[0] << " "
              << planeCoefficients->values[1] << " "
              << planeCoefficients->values[2] << " "
              << planeCoefficients->values[3] << std::endl;


//    cout << "Size of all point cloud is : " << inputPointCloud->points.size() << endl;
//    std::cerr << "Model PlaneInliers: " << planeInliers->indices.size () << std::endl;

    calculatePlaneVector();

    std::sort(planeInliers->indices.begin(), planeInliers->indices.end());

    int planeIndexCounter = 0;

    for(int i=0 ; i< inputPointCloud->points.size();i++) {
        if(i >= planeInliers->indices[planeIndexCounter]) {
            planeIndexCounter++;
            planePointcloud_ptr->points.push_back(inputPointCloud->points[i]);
            continue;
        }
        noPlanePointcloud_ptr->points.push_back(inputPointCloud->points[i]);
    }

    return true;
}

void PlaneProcessor::calculatePlaneVector() {

    const float ax = planeCoefficients->values[0];
    const float ay = planeCoefficients->values[1];
    const float az = planeCoefficients->values[2];
    cout << "planeVector : " << planeCoefficients->values[0] << " , " << planeCoefficients->values[1]  << " , " << planeCoefficients->values[2] << " , " << planeCoefficients->values[3]<< endl;
    Eigen::Vector3f planeVector(ax,ay,az);

    setPlaneVector(planeVector);

}


PlaneProcessor::PlaneProcessor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_point_cloud_ptr, bool &bSetOptimization) {

    pcl::ModelCoefficients::Ptr _planeCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr _planeInliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _planePointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _noPlanePointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    planeSegmenter.setOptimizeCoefficients (bSetOptimization);
    // Mandatory
    planeSegmenter.setModelType (pcl::SACMODEL_PLANE);
    planeSegmenter.setMethodType (pcl::SAC_RANSAC);
    planeSegmenter.setDistanceThreshold (0.3);
    planeSegmenter.setInputCloud (_point_cloud_ptr);

    setInputPointCloud(_point_cloud_ptr);
    setPlaneCoefficient(_planeCoefficients);
    setPlaneInliers(_planeInliers);
    setPlanePointCloud_ptr(_planePointcloud);
    setNoPlanePointCloud_ptr(_noPlanePointcloud);


}

void PlaneProcessor::setPlaneVector(Eigen::Vector3f &_planeVector) {
    planeVector = _planeVector;
}

Eigen::Vector3f PlaneProcessor::getPlaneVector() {
    return planeVector;
}

void PlaneProcessor::setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_point_cloud_ptr) {
    inputPointCloud = _point_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneProcessor::getInputPointCloud() {
    return inputPointCloud;
}

void PlaneProcessor::setPlaneCoefficient(pcl::ModelCoefficients::Ptr &_planeCoefficients) {
    planeCoefficients = _planeCoefficients;
}

pcl::ModelCoefficients::Ptr PlaneProcessor::getPlaneCoefficient() {
    return planeCoefficients;
}

void PlaneProcessor::setPlaneInliers(pcl::PointIndices::Ptr &_planeInliers) {
    planeInliers = _planeInliers;
}

pcl::PointIndices::Ptr PlaneProcessor::getPlaneInliers() {
    return planeInliers;
}

void PlaneProcessor::setPlanePointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_planePointcloud) {
    planePointcloud_ptr = _planePointcloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneProcessor::getPlanePointCloud_ptr() {
    return planePointcloud_ptr;
}

void PlaneProcessor::setNoPlanePointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_noPlanePointcloud) {
    noPlanePointcloud_ptr = _noPlanePointcloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneProcessor::getNoPlanePointCloud_ptr() {
    return noPlanePointcloud_ptr;
}