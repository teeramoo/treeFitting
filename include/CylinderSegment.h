//
// Created by teeramoo on 25/4/2561.
//

#ifndef TREEFIT_CYLINDERSEGMENT_H
#define TREEFIT_CYLINDERSEGMENT_H

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

#include <NormalEstimator.h>

class CylinderSegment {
public:

    CylinderSegment();
    CylinderSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Eigen::Vector3f &normalVector, double &epsAngle, bool &bOptimization);

    ~CylinderSegment();

    void performSegmentation(pcl::PointXYZRGB &refKeyframe);

    void setCylinderCoefficient(pcl::ModelCoefficients::Ptr &_cylinderCoefficient);
    pcl::ModelCoefficients::Ptr getCylinderCoefficient();

    void setCylinderInliers(pcl::PointIndices::Ptr &_cylinderInliers);
    pcl::PointIndices::Ptr getCylinderInliers();

    void setCylinderPointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cylinderPointCloud_ptr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCylinderPointcloud_ptr();


private:

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> cylinderSegmentator;
    pcl::ModelCoefficients::Ptr cylinderCoefficients;
    pcl::PointIndices::Ptr cylinderInliers;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderPointcloud_ptr;

};


#endif //TREEFIT_CYLINDERSEGMENT_H
