//
// Created by teeramoo on 2/8/2561.
//

#ifndef TREEFIT_HEIGHTESTIMATOR_H
#define TREEFIT_HEIGHTESTIMATOR_H

#endif //TREEFIT_HEIGHTESTIMATOR_H


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
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

class HeightEstimator{
public:

    HeightEstimator(pcl::ModelCoefficientsPtr &_cylinderCoefficient);
    ~HeightEstimator();

    int measureHeight();

};