//
// Created by teeramoo on 25/4/2561.
//

#include "CylinderSegment.h"

CylinderSegment::~CylinderSegment() {
    cout << "Cylinder segmentation has been destroyed." << endl;
}

CylinderSegment::CylinderSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Eigen::Vector3f &normalVector,
                                 double &epsAngle, bool &bOptimization) {

    cylinderSegmentator.setOptimizeCoefficients(bOptimization);
    cylinderSegmentator.setModelType (pcl::SACMODEL_CYLINDER);
    cylinderSegmentator.setAxis(normalVector);
    cylinderSegmentator.setEpsAngle(epsAngle);
    cylinderSegmentator.setMethodType (pcl::SAC_RANSAC);
    cylinderSegmentator.setNormalDistanceWeight (0.0);
    cylinderSegmentator.setMaxIterations (50000);
    cylinderSegmentator.setDistanceThreshold (0.2);
    cylinderSegmentator.setRadiusLimits (0.01, 0.1);
    cylinderSegmentator.setInputCloud (inputPointCloud);

    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderPointCloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    setCylinderInliers(inliers_cylinder);
    setCylinderCoefficient(coefficients_cylinder);
    setCylinderPointCloud_ptr(cylinderPointCloud_ptr);

    NormalEstimator normalEstimator(inputPointCloud,1.0);
    normalEstimator.calculate();
    cylinderSegmentator.setInputNormals (normalEstimator.getCloudNormal());
}

void CylinderSegment::performSegmentation(pcl::PointXYZRGB &refKeyframe) {
//    cout << "about to segment a cylinder" << endl;
    cylinderSegmentator.segment(*cylinderInliers,*cylinderCoefficients);
//    cout << "done segmenting a cylinder" << endl;

    if(cylinderCoefficients->values.size() == 0 or cylinderInliers->indices.size() < 60 ) {

        cylinderCoefficients->header.frame_id = "SKIP";
        cylinderPointcloud_ptr = NULL;
        cylinderInliers->indices.erase(cylinderInliers->indices.begin(),cylinderInliers->indices.end());

        cout << "skip adding a cylinder" << endl;
        return;
    }

    double xKF = refKeyframe.x;
    double yKF = refKeyframe.y;
    double zKF = refKeyframe.z;
    double xCylinder = cylinderCoefficients->values[0];
    double yCylinder = cylinderCoefficients->values[1];
    double zCylinder = cylinderCoefficients->values[2];

    double dDistance = sqrt(pow(xKF-xCylinder,2) + pow(yKF-yCylinder,2) + pow(zKF-zCylinder,2) );
//    cout << "distance is " << dDistance << endl;

    if(dDistance > 2.0) {
        cylinderCoefficients->header.frame_id = "SKIP";
        cylinderPointcloud_ptr = NULL;
        cylinderInliers->indices.erase(cylinderInliers->indices.begin(),cylinderInliers->indices.end());

//        if(cylinderInliers->indices.size() > 0)
//            sleep(1);

        cout << "skip adding cylinder due to too far distance " << endl;
        return;
    }

    cout << "Cylinder coefficient is : " << *cylinderCoefficients << endl;

    //Extract point cloud representing the cylinder
    pcl::ExtractIndices<pcl::PointXYZRGB> extract (true);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCylinderCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    extract.setInputCloud (cylinderSegmentator.getInputCloud());
    extract.setIndices (cylinderInliers);
    extract.setNegative (false);
    extract.filter(*cylinderPointcloud_ptr);

    cout << "Size of cylinderPointcloud_ptr is : " << cylinderPointcloud_ptr->points.size() << endl;

}

void CylinderSegment::setCylinderCoefficient(pcl::ModelCoefficients::Ptr &_cylinderCoefficient) {
    cylinderCoefficients = _cylinderCoefficient;
}

pcl::ModelCoefficients::Ptr CylinderSegment::getCylinderCoefficient() {
    return cylinderCoefficients;
}

void CylinderSegment::setCylinderInliers(pcl::PointIndices::Ptr &_cylinderInliers) {
    cylinderInliers = _cylinderInliers;
}

pcl::PointIndices::Ptr CylinderSegment::getCylinderInliers() {
    return cylinderInliers;
}

void CylinderSegment::setCylinderPointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cylinderPointCloud_ptr) {
    cylinderPointcloud_ptr = _cylinderPointCloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CylinderSegment::getCylinderPointcloud_ptr() {
    return cylinderPointcloud_ptr;
}
