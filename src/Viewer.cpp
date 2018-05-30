//
// Created by teeramoo on 24/4/2561.
//

#include "Viewer.h"

Viewer::Viewer() {}

Viewer::~Viewer() {

}

void Viewer::setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer) {
    cloudViewer = viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer::getViewer() {
    return cloudViewer;
}

Viewer::Viewer(std::string &viewerName, std::string &windowName) {

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer (viewerName));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setWindowName(windowName);

    setViewer(viewer);
}

void Viewer::addPlane(pcl::ModelCoefficients::Ptr planeCoefficients, std::string &planeName) {
    cloudViewer->addPlane(*planeCoefficients, planeName);
}

void Viewer::addPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud_ptr, std::string &pointCloudName) {
    cloudViewer->addPointCloud<pcl::PointXYZRGB>(pointCloud_ptr, pointCloudName);
}

void Viewer::addCylinder(pcl::ModelCoefficients::Ptr cylinderCoefficients, std::string &cylinderName) {
    cloudViewer->addCylinder(*cylinderCoefficients,cylinderName);
}

void Viewer::addNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointCloud,
                        pcl::PointCloud<pcl::Normal>::Ptr _cloud_normals, int displayLevel, double normalArrowScale,
                        std::string &normalName) {

    cloudViewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(_pointCloud, _cloud_normals, displayLevel, normalArrowScale, normalName);

}

void Viewer::addArrow(pcl::PointXYZRGB &keypoint1, pcl::PointXYZRGB &keypoint2,
                      double red, double green, double blue, bool showArrowLength,
                      std::string arrowName) {
    cloudViewer->addArrow(keypoint1,keypoint2,red,green,blue,arrowName);

}

void Viewer::addSphere(pcl::PointXYZRGB &center, double &radius, std::string &sphereName) {
    cout << "adding sphere" <<endl;
    cloudViewer->addSphere(center,radius,sphereName);
    cout << "done adding sphere" <<endl;
}

void Viewer::addLineByCoefficient(pcl::ModelCoefficients &lineCoefficients, std::string lineName) {
    cloudViewer->addLine(lineCoefficients,lineName);
}


void Viewer::run() {

    while (!cloudViewer->wasStopped ()) {
        cloudViewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}