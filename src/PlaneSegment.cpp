//
// Created by teeramoo on 23/4/2561.
//

#include "PlaneSegment.h"

PlaneSegment::~PlaneSegment() {
    cout << "PlaneSegment has been destroyed." << endl;
}


bool PlaneSegment::performSegmentation() {

    cout << "about to segment a plane" << endl;
    planeSegmentator.segment(*planeInliers, *planeCoefficients);
    cout << "finish segmentation" << endl;

    if (planeInliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model from the given dataset.");

        return false;
    }

    std::cerr << "Plane model coefficients: " << planeCoefficients->values[0] << " "
              << planeCoefficients->values[1] << " "
              << planeCoefficients->values[2] << " "
              << planeCoefficients->values[3] << std::endl;


    cout << "Size of all point cloud is : " << inputPointCloud->points.size() << endl;
    std::cerr << "Model PlaneInliers: " << planeInliers->indices.size () << std::endl;

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

void PlaneSegment::calculatePlaneVector() {

    const float ax = planeCoefficients->values[0];
    const float ay = planeCoefficients->values[1];
    const float az = planeCoefficients->values[2];
    cout << "planeCoefficient : " << planeCoefficients->values[0] << " , " << planeCoefficients->values[1]  << " , " << planeCoefficients->values[2] << " , " << planeCoefficients->values[3]<< endl;
    Eigen::Vector3f planeVector(ax,ay,az);

    setPlaneVector(planeVector);

}


PlaneSegment::PlaneSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_point_cloud_ptr, bool &bSetOptimization) {

    pcl::ModelCoefficients::Ptr _planeCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr _planeInliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _planePointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _noPlanePointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    planeSegmentator.setOptimizeCoefficients (bSetOptimization);
    // Mandatory
    planeSegmentator.setModelType (pcl::SACMODEL_PLANE);
    planeSegmentator.setMethodType (pcl::SAC_RANSAC);
    planeSegmentator.setDistanceThreshold (0.4);
    planeSegmentator.setInputCloud (_point_cloud_ptr);

    setInputPointCloud(_point_cloud_ptr);
    setPlaneCoefficient(_planeCoefficients);
    setPlaneInliers(_planeInliers);
    setPlanePointCloud_ptr(_planePointcloud);
    setNoPlanePointCloud_ptr(_noPlanePointcloud);


}

void PlaneSegment::setPlaneVector(Eigen::Vector3f &_planeVector) {
    planeVector = _planeVector;
}

Eigen::Vector3f PlaneSegment::getPlaneVector() {
    return planeVector;
}

void PlaneSegment::setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_point_cloud_ptr) {
    inputPointCloud = _point_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneSegment::getInputPointCloud() {
    return inputPointCloud;
}

void PlaneSegment::setPlaneCoefficient(pcl::ModelCoefficients::Ptr &_planeCoefficients) {
    planeCoefficients = _planeCoefficients;
}

pcl::ModelCoefficients::Ptr PlaneSegment::getPlaneCoefficient() {
    return planeCoefficients;
}

void PlaneSegment::setPlaneInliers(pcl::PointIndices::Ptr &_planeInliers) {
    planeInliers = _planeInliers;
}

pcl::PointIndices::Ptr PlaneSegment::getPlaneInliers() {
    return planeInliers;
}

void PlaneSegment::setPlanePointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_planePointcloud) {
    planePointcloud_ptr = _planePointcloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneSegment::getPlanePointCloud_ptr() {
    return planePointcloud_ptr;
}

void PlaneSegment::setNoPlanePointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_noPlanePointcloud) {
    noPlanePointcloud_ptr = _noPlanePointcloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneSegment::getNoPlanePointCloud_ptr() {
    return noPlanePointcloud_ptr;
}