//
// Created by teeramoo on 24/4/2561.
//

#ifndef TREEFIT_VIEWER_H
#define TREEFIT_VIEWER_H

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

//cpp libraries
#include <string>

class Viewer {

public:

    Viewer();
    Viewer(std::string &viewerName, std::string &windowName);

    ~Viewer();

    void setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer();

    void run();

    void addPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_ptr, std::string &pointCloudName);
    void addPlane(pcl::ModelCoefficients::Ptr planeCoefficients, std::string &planeName);
    void addCylinder(pcl::ModelCoefficients::Ptr cylinderCoefficients, std::string &cylinderName);
    void addNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointCloud, pcl::PointCloud<pcl::Normal>::Ptr _cloud_normals, int displayLevel,
                    double normalArrowScale, std::string &normalName);

    void addArrow(pcl::PointXYZRGB &keypoint1, pcl::PointXYZRGB &keypoint2,
                   double red, double green, double blue, bool showArrowLength, std::string arrowName);

    void addSphere(pcl::PointXYZRGB &center, double &radius, std::string &sphereName);

    void addLineByCoefficient(pcl::ModelCoefficients &lineCoefficients, std::string lineName);

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloudViewer;

};


#endif //TREEFIT_VIEWER_H
