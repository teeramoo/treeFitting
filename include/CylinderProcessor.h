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

class Cylinder
{
public:

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pointCloud;
    pcl::ModelCoefficients::Ptr cylinderCoef;
    pcl::PointIndices::Ptr cylinderInliers;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderPointCloud_ptr;

};
/*
struct comparePoints {

    bool operator() (pcl::PointXYZRGB &p1, pcl::PointXYZRGB &p2) {
        if (p1.x != p2.x)
            return p1.x > p2.x;
        else if (p1.y != p2.y)
            return  p1.y > p2.y;
        else
            return p1.z > p2.z;
    }
};
*/

class CylinderProcessor {
public:

    enum refineOptions{
        XZ,
        XYZ
    };

    struct sortStruct {
        CylinderProcessor* sorter;
        sortStruct(CylinderProcessor* _sorter) : sorter(_sorter) {};

        bool operator() (pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {

            if (p1.x != p2.x)
                return p1.x > p2.x;
            else if (p1.y != p2.y)
                return  p1.y > p2.y;
            else
                return p1.z > p2.z;

        }

    };

    struct equalPointStruct {
        CylinderProcessor* equalChecker;
        equalPointStruct(CylinderProcessor* _equalChecker) : equalChecker(_equalChecker) {};

        bool operator() (pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {

            if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
                return true;
            return false;

        }

    };




    CylinderProcessor();
    ~CylinderProcessor();

    // Instantiator for cylinder segmentation
    CylinderProcessor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Eigen::Vector3f &normalVector, double &epsAngle, bool &bOptimization);

    // Instatiator for cylinder refinement
    CylinderProcessor(double &distanceLimit, enum CylinderProcessor::refineOptions refineOption);

    void cluster(std::vector<Cylinder> &allCylinders,std::vector<std::vector<Cylinder>> &possibleGroupCylinders );

    void segment(pcl::PointXYZRGB &refKeyframe, bool &bCheckDistance);
    void segment();


    void removeDuplicates(std::vector<std::vector<Cylinder>> &possibleGroupCylinders, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &allPointsAfterCulling);

    bool cloud2vec(std::vector<std::vector<Cylinder>> &possibleGroupCylinders, std::vector<std::vector<pcl::PointXYZRGB>> &vecCylinderPoints);

    void sortPoints(std::vector<pcl::PointXYZRGB> vecPoints);

    void equalPoint(std::vector<pcl::PointXYZRGB> vecSortedPoints, int &uniqueEndIndex);



    void setCylinderCoefficient(pcl::ModelCoefficients::Ptr &_cylinderCoefficient);
    pcl::ModelCoefficients::Ptr getCylinderCoefficient();

    void setCylinderInliers(pcl::PointIndices::Ptr &_cylinderInliers);
    pcl::PointIndices::Ptr getCylinderInliers();

    void setCylinderPointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cylinderPointCloud_ptr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCylinderPointcloud_ptr();



private:

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> cylinderSegmenter;
    pcl::ModelCoefficients::Ptr cylinderCoefficients;
    pcl::PointIndices::Ptr cylinderInliers;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderPointcloud_ptr;

    enum CylinderProcessor::refineOptions refineOption;
    double distanceLimit;

};


#endif //TREEFIT_CYLINDERSEGMENT_H
