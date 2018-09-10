/* \author Geoffrey Biggs */


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
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>


// cpp libraries
#include <chrono>
#include <ctime>
#include <tuple>        
#include <iostream>
#include <string>
#include <fstream>

#include <PlaneProcessor.h>
#include <Viewer.h>
#include <NormalEstimator.h>
#include <CylinderProcessor.h>
#include<bits/stdc++.h>
#include <iostream>
#include <utility>
#include <functional>
#include <math.h>

#include "opencv2/core/core.hpp"

class Tree
{
public:
    pcl::ModelCoefficients::Ptr cylinderCoef;
    pcl::ModelCoefficients::Ptr relatedPlaneCoef;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trunkPointcloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr treetopPointcloud;

    bool isTree;
    double radius;
    double height;
    pcl::PointXYZRGB projectedPointOnPlane;
    pcl::PointXYZRGB highestPoint;

};

class GroundTruth {

public:
    int numTree;
    double circumference;
    double height;
};

void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}


void readFarmInformation(std::string sFarmPath, double &plantDistanceColumn, double &plantDistanceRow);
void readInputFile(std::string inputFile, std::string &sDownPointCloud, std::string &sUpPointCloud, std::string &sDownKF,double &searchRadius,
                   double &dEpAngle, double &plantDistanceColumn, double &plantDistanceRow, std::string &sPathToGroundTruth);

bool comparePoint(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2);
bool equalPoint(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2);
void tokenize(const std::string &str, std::vector<std::string> &vTokens);
void readCSV(std::string &_pathToCSVfile, std::vector<GroundTruth> &_vGroundTruth);
void calculatePlaneNormalVector(Eigen::Vector4d &planeEquation, Eigen::Vector3d &planeNormalVector);
void projectPointsToPlane(Eigen::Vector4d &planeEquation, Eigen::Vector3d &planeNormalVector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud);

void projectPairPointIndexToPlane(Eigen::Vector4d &planeEquation, Eigen::Vector3d &planeNormalVector, std::vector<std::pair<pcl::PointXYZRGB, int>> &vPairPointIndex,
                                  std::vector<std::pair<pcl::PointXYZRGB, int>> &vProjectedPairPointIndex);

void searchRadius2d(Eigen::Vector4d &planeEquation, Eigen::Vector3d &planeNormalVector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &projectedKeypoints,
                    std::vector<std::pair<pcl::PointXYZRGB,int>> &vProjectedPairPointIndex,
                    double &plantDistanceRow, double &plantDistanceColumn,
                    std::vector<std::pair<pcl::PointXYZRGB,int>> &vCleanedProjectedPairPointIndex);

void extractClusterProjectedIndices(std::vector<pcl::PointIndices> &cluster_indices,  std::vector<std::pair<pcl::PointXYZRGB,int>> &vCleanedProjectedPairPointIndex,
                                    std::vector<std::vector<int>> &vClusteredProjectedPointsIndices);

void get3dPointFrom2dIndices(std::vector<std::vector<int>> &vClusteredProjectedPointsIndices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &originalCloud ,
                             std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vPointClouds);

void cloudClustering(Eigen::Vector4d &planeEquation, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud,
                     double &plantDistanceRow, double &plantDistanceColumn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
                     std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vOutputClusteredCloud);

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv) {


    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //std::vector<int> vkpCounter;
    double tkpx = 0, tkpy = 0, tkpz = 0;
   

    if(argc != 2) {
        cout << "Usage : " << argv[0] << "<Input_file>" << endl;
        return -1;
    }

    //Get inputs
    std::string sInputFile = std::string(argv[1]);
    std::string sDownwardInput;
    std::string sUpwardInput;
    std::string sDownwardKeyframe;
    double dSearchRadius;
    double dEpsAngle;
    double plantDistanceColumn;
    double plantDistanceRow;
    std::string sPathToGroundTruth;


    readInputFile(sInputFile, sDownwardInput, sUpwardInput, sDownwardKeyframe, dSearchRadius, dEpsAngle, plantDistanceColumn, plantDistanceRow, sPathToGroundTruth );


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downward_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoint_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr upward_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Read input clouds
    //Input cloud will have pcd format and color in float bits (reinterpret cast from uint32_t)
    pcl::io::loadPCDFile(sDownwardInput, *downward_point_cloud_ptr);
    pcl::io::loadPCDFile(sDownwardKeyframe, *keypoint_ptr);
    pcl::io::loadPCDFile(sUpwardInput, *upward_point_cloud_ptr);


    //View inputs
    std::string inputViewerName = "original inputs";
    Viewer inputViewer(inputViewerName, inputViewerName);

    std::string inputDownwardPointCloudName = "inputDownwardPoint1";
    std::string inputUpwardPointCloudName = "inputUpwardPoint1";
    inputViewer.addPointcloud(downward_point_cloud_ptr, inputDownwardPointCloudName);
    inputViewer.addPointcloud(upward_point_cloud_ptr, inputUpwardPointCloudName);

    inputViewer.run();
  
    downward_point_cloud_ptr->width = (int) downward_point_cloud_ptr->points.size();
    downward_point_cloud_ptr->height = 1;
    keypoint_ptr->width = (int) keypoint_ptr->points.size();
    keypoint_ptr->height = 1;
    upward_point_cloud_ptr->width = (int) upward_point_cloud_ptr->points.size();
    upward_point_cloud_ptr->height = 1;

    pcl::PCDWriter writer;

    //Start segmenting a plane
    pcl::ModelCoefficients::Ptr PlaneCoefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr PlaneInliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr noPlanePointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    bool bPlaneOptimization = false;
    PlaneProcessor myPlane = PlaneProcessor(downward_point_cloud_ptr, bPlaneOptimization);

    if (!myPlane.segment()) {

        cout << "Can't get a plane model from the given point cloud" << endl;
        return -1;
    }


    //Debugging
    cout << "Total points without plane is : " << myPlane.getNoPlanePointCloud_ptr()->points.size() << endl;
    cout << "Total points considered as a plane : " << myPlane.getPlanePointCloud_ptr()->points.size() << endl;
    cout << "Comparison between all points and plane + without plane : " << myPlane.getInputPointCloud()->points.size()
         << " | " << myPlane.getNoPlanePointCloud_ptr()->points.size() + myPlane.getPlanePointCloud_ptr()->points.size()
         << endl;

    // Visualize plane
    std::string planeViewerName = "plane1";
    Viewer planeViewer(planeViewerName, planeViewerName);

    std::string planePointCloudName = "planePoint1";
    planeViewer.addPointcloud(myPlane.getPlanePointCloud_ptr(), planePointCloudName);

    planeViewer.addPlane(myPlane.getPlaneCoefficient(), planeViewerName);

    planeViewer.run();

    //End of plane segmentation


    NormalEstimator planeNormals(myPlane.getNoPlanePointCloud_ptr(), dSearchRadius);
    planeNormals.calculate();

    cout << "calculate plane normal" << endl;

    std::string noPlaneNormalName = "no plane normals";
    Viewer noPlaneNormalViewer(noPlaneNormalName, noPlaneNormalName);
    std::string noPlanePointCloudName = "no plane PC";
    noPlaneNormalViewer.addPointcloud(planeNormals.getInputPointCloud(), noPlanePointCloudName);
    noPlaneNormalViewer.addNormals(planeNormals.getInputPointCloud(), planeNormals.getCloudNormal(), 10, 0.05,
                                   noPlaneNormalName);

    noPlaneNormalViewer.run();

    std::string be4name = "b4name";
    Viewer beforeProjection(be4name,be4name);
    std::string b4projection = "b4projection";
    beforeProjection.addPointcloud(myPlane.getNoPlanePointCloud_ptr(),b4projection);
    beforeProjection.run();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputProjectedPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
    inputProjectedPoints = myPlane.getNoPlanePointCloud_ptr();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectedPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vClusteredPointCloud;

    // cant use plane equation, must use normal vector instead
    Eigen::Vector4d planeEquation(myPlane.getPlaneCoefficient()->values[0],myPlane.getPlaneCoefficient()->values[1],
                                  myPlane.getPlaneCoefficient()->values[2],myPlane.getPlaneCoefficient()->values[3]);


    cloudClustering(planeEquation, inputProjectedPoints, plantDistanceRow, plantDistanceColumn, keypoint_ptr, vClusteredPointCloud);

    std::string afterName = "afterName";
    Viewer afterProjection(afterName,afterName);
    std::string aftProjection = "aftProjection";
    afterProjection.addPointcloud(projectedPoints,aftProjection);
    afterProjection.run();

    double epsAngleS = dEpsAngle/180*M_PI;
    bool bCylinderOptimization = false;
    Eigen::Vector3d tempRefAxis;
    calculatePlaneNormalVector(planeEquation, tempRefAxis);
    Eigen::Vector3f refCylinderAxis( (float) tempRefAxis[0], (float) tempRefAxis[1], (float) tempRefAxis[2]);

    cout << "tempRefAxis : " << tempRefAxis << endl;

    cout << "refCylinderAxis : " << refCylinderAxis << endl;

    std::vector<Tree> vTrees;

    std::string treeViewName = "treeViewName";
    Viewer treeViewer(treeViewName,treeViewName);

    for(int clusterNumber = 0; clusterNumber < vClusteredPointCloud.size(); clusterNumber++) {

        CylinderProcessor tempCylinder(vClusteredPointCloud[clusterNumber], refCylinderAxis, epsAngleS, bCylinderOptimization);
        tempCylinder.segment();

        Tree tempTree;
        //Check if the algorithm can find any tree in it
        if(tempCylinder.getCylinderCoefficient()->values.empty()) {

            tempCylinder.getCylinderCoefficient()->header.frame_id = "SKIP";
            tempCylinder.getCylinderInliers()->indices.clear();

            tempTree.isTree = false;
            tempTree.height = 0.0;
            tempTree.radius = 0.0;

        } else {

            tempTree.isTree = true;
            tempTree.cylinderCoef = tempCylinder.getCylinderCoefficient();
            tempTree.radius = tempCylinder.getCylinderCoefficient()->values[6];
            tempTree.trunkPointcloud = tempCylinder.getCylinderPointcloud_ptr();

            std::string cylinderName = "cyName" + std::to_string(clusterNumber);
            treeViewer.addCylinder(tempTree.cylinderCoef, cylinderName);

            std::string pointcloudName = "pcName" + std::to_string(clusterNumber);
            treeViewer.addPointcloud(vClusteredPointCloud[clusterNumber], pointcloudName);
        }

        vTrees.push_back(tempTree);

    }

    treeViewer.run();
    /*
// Identify tree location
    //Project the center of the tree to the ground plane ( 3D plane equation )
    cout << "starting identify tree location" << endl;
    //create point cloud that has 3D points equal to number of cylinder found
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trees3Dpoint (new pcl::PointCloud<pcl::PointXYZRGB>);
    trees3Dpoint->width = vecRefinedCylinders.size();
    trees3Dpoint->height = 1;
    trees3Dpoint->points.resize(trees3Dpoint->width * trees3Dpoint->height);

    // convert normal vector of the plane to Eigen::Vector

    //Project the center of the cylinder of each tree along its axis vector to the ground (3D plane)
    for( size_t i = 0; i < foundTrees.size(); i++) {

        Eigen::Vector3d plane(foundTrees[i].relatedPlaneCoef->values[0],
                              foundTrees[i].relatedPlaneCoef->values[1],
                              foundTrees[i].relatedPlaneCoef->values[2]);


        Eigen::Vector3d pointOnLine(foundTrees[i].cylinderCoef->values[0],
                                    foundTrees[i].cylinderCoef->values[1],
                                    foundTrees[i].cylinderCoef->values[2]);


        Eigen::Vector3d line(foundTrees[i].cylinderCoef->values[3],
                             foundTrees[i].cylinderCoef->values[4],
                             foundTrees[i].cylinderCoef->values[5]);

        cout << "value of PointOnLine is : " << pointOnLine << endl;

        //calculate dot product between line and plane
        double lineDotPlane = line.dot(plane);
        cout << "value of lineDotPlane is : " << lineDotPlane << endl;
        if(lineDotPlane == 0) { // line and plane does not intersect to each other.
            cout << "lineDotPlane is zero " << endl;
            continue;
        }

        double t = - (plane[0]*pointOnLine[0] + plane[1]*pointOnLine[1] + plane[2]*pointOnLine[2] + myPlane.getPlaneCoefficient()->values[3]) / lineDotPlane;

        cout << "value of t is " << t << endl;
        cout << "value of line is : " << line << endl;
        double tempX = pointOnLine[0] + line[0]*t;
        double tempY = pointOnLine[1] + line[1]*t;
        double tempZ = pointOnLine[2] + line[2]*t;

        cout << "projected temp Value is : " << tempX << " , " << tempY << " , " << tempZ << endl;
        //round value of tempX, tempY, tempZ

        pcl::PointXYZRGB projectedPoint;
        projectedPoint.x = tempX;
        projectedPoint.y = tempY;
        projectedPoint.z = tempZ;

        cout << "projected point is : " << projectedPoint.x << " , " << projectedPoint.y << " , " << projectedPoint.z << endl;
        // Add the projected point to the cloud
        foundTrees[i].projectedPointOnPlane = projectedPoint;
        cout << "value of foundTrees[i].projectedPointOnPlane is : " << foundTrees[i].projectedPointOnPlane << endl;
    }

*/
    return -1;

/*
    //remove all 3D points or the right side of the key points (incrementally remove the points every 10 keyframes)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudWithoutPlane(new pcl::PointCloud<pcl::PointXYZRGB>);
    pointcloudWithoutPlane = myPlane.getNoPlanePointCloud_ptr();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr leftPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector3f planeNormalVector(myPlane.getPlaneVector()[0],myPlane.getPlaneVector()[1],myPlane.getPlaneVector()[2]);

    double iteRound = round(keypoint_ptr->points.size()/10);
 //   if(keypoint_ptr->points.size() % 10 != 0)
 //       iteRound +=1;
    cout << iteRound << endl;

    removePointsRightSide(keypoint_ptr->points.front(), keypoint_ptr->points.back(),
                          planeNormalVector,pointcloudWithoutPlane, leftPointCloud);


    for(size_t i = 0; i < (int) iteRound; i++) {

        removePointsRightSide(keypoint_ptr->points[i*10], keypoint_ptr->points[i*10 + 9],
                              planeNormalVector,pointcloudWithoutPlane, leftPointCloud);


        std::string afterRemoval = "after removal";
        std::string afterPlaneName = "noRightPlaneName";

        Viewer removeRightPointsAfter(afterRemoval, afterRemoval);

        removeRightPointsAfter.addPointcloud(leftPointCloud, afterPlaneName);
          std::string aName = "anme";
        //  removeRightPointsAfter.addArrow(keypoint_ptr->points[round(keypoint_ptr->points.size()/10)*10], keypoint_ptr->points.back(),255,255,255,false,aName);
        removeRightPointsAfter.addArrow(keypoint_ptr->points[i*10],keypoint_ptr->points[i*10 + 9],0,255,0,false, aName + std::to_string(i));
        removeRightPointsAfter.run();


    }





    std::string beforeRemoval = "before removal";
    std::string noPlaneName = "noPlaneName";
    std::string aName = "anme";

    Viewer removeRightPointsBefore(beforeRemoval, beforeRemoval);
    removeRightPointsBefore.addPointcloud(myPlane.getNoPlanePointCloud_ptr(),noPlaneName);
    removeRightPointsBefore.addArrow(keypoint_ptr->points.front(), keypoint_ptr->points.back(),255,255,255,false,aName);
    removeRightPointsBefore.run();


    std::string afterRemoval = "after removal";
    std::string afterPlaneName = "noRightPlaneName";

    Viewer removeRightPointsAfter(afterRemoval, afterRemoval);

    removeRightPointsAfter.addPointcloud(leftPointCloud, afterPlaneName);

    removeRightPointsAfter.addArrow(keypoint_ptr->points.front(), keypoint_ptr->points.back(),255,255,255,false,aName);
    removeRightPointsAfter.run();

*/
    std::vector<Cylinder> allCylinders;

    double epsAngle = dEpsAngle/180*M_PI;
    bool bInitCylinderOptimization = false;
    Eigen::Vector3f unitPlaneVector = myPlane.getPlaneVector();
    unitPlaneVector.normalized();

    Eigen::Vector3f firstKF(keypoint_ptr->points[1].x,
                            keypoint_ptr->points[1].y,
                            keypoint_ptr->points[1].z);

    Eigen::Vector3f lastKF(keypoint_ptr->points.back().x,
                           keypoint_ptr->points.back().y,
                           keypoint_ptr->points.back().z);

    Eigen::Vector3f unitVecFirstLastKF(lastKF - firstKF);
    unitVecFirstLastKF.normalized();

    //cross product of two vectors
    Eigen::Vector3f unitCrossVector = unitPlaneVector.cross(unitVecFirstLastKF);

    double cylinderRadius;

    if(plantDistanceColumn < plantDistanceRow)
        cylinderRadius = plantDistanceColumn*0.75;
    else
        cylinderRadius = plantDistanceRow*0.75;

    cout << "starting initial point cloud clustering" << endl;
    pcl::PointXYZRGB tempPoint;
    //3D points segmentation using keypoints as the center.
    cout << "total noPlane pointcloud : " << myPlane.getNoPlanePointCloud_ptr()->points.size() << endl;
    for (int i = 0; i < keypoint_ptr->points.size(); i++) {

        Cylinder tempCylinder;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_pointCloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);


        Eigen::Vector3f point1(keypoint_ptr->points[i].x + unitCrossVector[0]*plantDistanceRow*0.5,
                               keypoint_ptr->points[i].y + unitCrossVector[1]*plantDistanceRow*0.5,
                               keypoint_ptr->points[i].z + unitCrossVector[2]*plantDistanceRow*0.5);

        Eigen::Vector3f point2(point1[0] + unitPlaneVector[0]*3,
                               point1[1] + unitPlaneVector[1]*3,
                               point1[2] + unitPlaneVector[2]*3);

        Eigen::Vector3f point3(point1[0] - unitPlaneVector[0]*3,
                               point1[1] - unitPlaneVector[1]*3,
                               point1[2] - unitPlaneVector[2]*3);

        for(int j = 0; j < myPlane.getNoPlanePointCloud_ptr()->points.size(); j++) {

            tempPoint = myPlane.getNoPlanePointCloud_ptr()->points[j];

            Eigen::Vector3f testPoint(tempPoint.x,
                                      tempPoint.y,
                                      tempPoint.z);

            // qp1 = testPoint - point1
            Eigen::Vector3f qp1(testPoint - point1);

            // qp2 = testPoint - point2
            Eigen::Vector3f qp2(testPoint - point2);

            // p2p1 = point2 - point1
            Eigen::Vector3f p2p1(point2 - point1);

            // p1p2 = point1 - point2
            Eigen::Vector3f p1p2(point1 - point2);

            //1. Check if testPoint is between the two circular facets of the cylinder model
            // with (q-p1) dot (p2-p1) >=0 && (q-p2) dot (p1-p2) >=0
            if( qp1.dot(p2p1) < 0 or qp2.dot(p1p2) <0 )
                continue;

            //2. Check if norm(qp1 x p2p1) <= contraintRadius * norm(p2p1)
            Eigen::Vector3f Crossqp1xp2p1( qp1.cross(p2p1) );

            if(Crossqp1xp2p1.norm() > cylinderRadius * p2p1.norm() )
                continue;

            cout << "added point to the vector" << endl;
            temp_pointCloud_ptr->points.push_back(tempPoint);

        }

        tempCylinder.input_pointCloud = temp_pointCloud_ptr;
        cout << "There are " << tempCylinder.input_pointCloud->points.size() << " points for keyframe number " << i << endl;


        CylinderProcessor initialCylinderSegment(temp_pointCloud_ptr,unitPlaneVector, epsAngle, bInitCylinderOptimization);

        bool checkDistance = false;
        initialCylinderSegment.segment(keypoint_ptr->points[i], checkDistance); // Dont check distance

        tempCylinder.cylinderCoef = initialCylinderSegment.getCylinderCoefficient();
        tempCylinder.cylinderInliers = initialCylinderSegment.getCylinderInliers();
        tempCylinder.cylinderPointCloud_ptr = initialCylinderSegment.getCylinderPointcloud_ptr();
        allCylinders.push_back(tempCylinder);

/*
        pcl::PointXYZRGB tempKeyPoint = keypoint_ptr->points[i];
        double tkpy = tempKeyPoint.y;
        double tkpx = tempKeyPoint.x;
        double tkpz = tempKeyPoint.z;

        Cylinder tempCylinder;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_pointCloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);


        for (int j = 0; j < downward_point_cloud_ptr->points.size(); j++) {
            pcl::PointXYZRGB point = downward_point_cloud_ptr->points[j];
            double tmpx = point.x;
            double tmpy = point.y;
            double tmpz = point.z;
            double dist = sqrt(pow(tkpx - tmpx, 2.0) + pow(tkpy - tmpy, 2.0) + pow(tkpz - tmpz, 2.0));

            if (dist < dPointDistance) {
                float tfrgb = point.rgb;
                uint32_t temprgb = *reinterpret_cast<uint32_t *>(&tfrgb);
                if (temprgb == 16777215)
                    continue;    //It's a keyframe point, skip it for now

                temp_pointCloud_ptr->points.push_back(point);
            }

        }


        tempCylinder.input_pointCloud = temp_pointCloud_ptr;
        cout << "There are " << tempCylinder.input_pointCloud->points.size() << " points for keyframe number " << i << endl;


        CylinderProcessor initialCylinderSegment(temp_pointCloud_ptr,planeVector, epsAngle, bInitCylinderOptimization);

        initialCylinderSegment.segment(keypoint_ptr->points[i]);

        tempCylinder.cylinderCoef = initialCylinderSegment.getCylinderCoefficient();
        tempCylinder.cylinderInliers = initialCylinderSegment.getCylinderInliers();
        tempCylinder.cylinderPointCloud_ptr = initialCylinderSegment.getCylinderPointcloud_ptr();
        allCylinders.push_back(tempCylinder);
*/

    }

    //3D points segmentation using keypoints as the center.


    std::string cylinderViewerName = "Show all cylinders";
    Viewer cylinderViewer(cylinderViewerName,cylinderViewerName);
    std::string cylinderPointCloudName = "cylinderPointCloud";

    cylinderViewer.addPointcloud(myPlane.getNoPlanePointCloud_ptr(),noPlanePointCloudName);

    cout << "start adding cylinders to viewer" <<endl;
    double sphereRadius = 0.12;
    int skipCounter = 0;
    std::vector<int> cylinderIndices;

    for(int i=0; i< allCylinders.size(); i++) {

        std::string cylinderName = "cylinder" + std::to_string(i);
        cylinderIndices.push_back(allCylinders[i].cylinderInliers->indices.size());

        if(allCylinders[i].cylinderInliers->indices.size() < 1) {
            skipCounter++;
            continue;
        }


     //   if(allCylinders[i].cylinderInliers->indices.size() >100)
        cylinderViewer.addCylinder(allCylinders[i].cylinderCoef, cylinderName);

        if(allCylinders[i].cylinderCoef->values.size() >0) {
            pcl::PointXYZRGB sphereCenter;
     //       cout <<"start making points" <<endl;
            sphereCenter.x = (float) allCylinders[i].cylinderCoef->values[0];
            sphereCenter.y = (float) allCylinders[i].cylinderCoef->values[1];
            sphereCenter.z = (float) allCylinders[i].cylinderCoef->values[2];
            std::string sphereName = "sphere" + std::to_string(i);
        //    cylinderViewer.addSphere(sphereCenter,sphereRadius,sphereName);

            pcl::ModelCoefficients lineCoefficients;
            lineCoefficients.values.push_back(allCylinders[i].cylinderCoef->values[0]);
            lineCoefficients.values.push_back(allCylinders[i].cylinderCoef->values[1]);
            lineCoefficients.values.push_back(allCylinders[i].cylinderCoef->values[2]);
            lineCoefficients.values.push_back(allCylinders[i].cylinderCoef->values[3]);
            lineCoefficients.values.push_back(allCylinders[i].cylinderCoef->values[4]);
            lineCoefficients.values.push_back(allCylinders[i].cylinderCoef->values[5]);

            std::string lineName = "line" + std::to_string(i);
//            cylinderViewer.addLineByCoefficient(lineCoefficients, lineName);

        }
        else {
            cout << "skip one cylinder" << endl;
        }

        if(allCylinders[i].cylinderCoef->header.frame_id == "SKIP")
            skipCounter++;

    }


    cout << "total number of cylinder found is : " << allCylinders.size() <<endl;
    cout << "total number of keyframe is : " << keypoint_ptr->points.size() << endl;
    cout << "total skipped : " << skipCounter << endl;
    pcl::PointXYZRGB keyPoint1 = keypoint_ptr->points.front();

/*
    for(int i=0;i< keypoint_ptr->points.size(); i++) {

        if(allCylinders[i].cylinderCoef->values.size() > 0) {
    // kd       pcl::PointXYZRGB keyPoint1 = keypoint_ptr->points[i];
            pcl::PointXYZRGB keyPoint2 = keypoint_ptr->points[i];
       //     keyPoint2.x =  allCylinders[i].cylinderCoef->values[0];
       //     keyPoint2.y =  allCylinders[i].cylinderCoef->values[1];
       //     keyPoint2.z =  allCylinders[i].cylinderCoef->values[2];
            std::string arrowName = "myArrow" + std::to_string(i);
            if(i% 10 == 0)
                cylinderViewer.addArrow(keyPoint1,keyPoint2,0,255,0,true,arrowName);
        }


    }

//    cylinderViewer.addPlane(myPlane.getPlaneCoefficient(), planeViewerName);
 */
    cylinderViewer.run();





    std::vector<std::vector<Cylinder>> possibleGroupCylinders;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> allPointsAfterCulling;

    double distanceLimit = 1.0;

    CylinderProcessor cylinderRefiner(distanceLimit, CylinderProcessor::XZ );
    cylinderRefiner.cluster(allCylinders, possibleGroupCylinders );


    cout << "there are " << possibleGroupCylinders.size() << " possible trees in the dataset." << endl;

    std::vector<std::vector<pcl::PointXYZRGB>> vecCylinderPoints;
    cylinderRefiner.cloud2vec(possibleGroupCylinders,vecCylinderPoints);
    cout << "vecCylinderPoint.size() is : " << vecCylinderPoints.size() << endl;
    pcl::PointCloud<pcl::PointXYZRGB> pointCloudCylinderAfterCulling;


    for(int i=0;i< vecCylinderPoints.size();i++) {

        std::sort(vecCylinderPoints[i].begin(), vecCylinderPoints[i].end(), comparePoint);

        auto unique_end = std::unique(vecCylinderPoints[i].begin(), vecCylinderPoints[i].end(), equalPoint);

        //Sorting problem --> Try displaying all of them vs points after culling
        cout << "Number of points before culling : " << vecCylinderPoints[i].size() << endl;

        vecCylinderPoints[i].erase(unique_end, vecCylinderPoints[i].end());

        cout << "Number of points after culling : " << vecCylinderPoints[i].size() << endl;


/*
        for(int j=0;j<vecCylinderPoints[i].size();j++) {

            vecCylinderPoints[i][j].r = 255;
            vecCylinderPoints[i][j].g = 0;
            vecCylinderPoints[i][j].b = 0;
            pointCloudCylinderAfterCulling.push_back(vecCylinderPoints[i][j]);

        }
        */
/*
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderPoints_ptrAfterCulling(&pointCloudCylinderAfterCulling);
        allPointsAfterCulling.push_back(cylinderPoints_ptrAfterCulling);
        cout << "cylinderPoints_ptrAfterCulling : " << cylinderPoints_ptrAfterCulling->points.size() << endl;

        for(int k=0; k< allPointsAfterCulling.size();k++) {
            cout << "allPointsAfterCulling[" << k <<"] has " << allPointsAfterCulling[i]->points.size() << " points." <<endl;

        }
*/
    }


    // create point cloud
    std::string cloudAfterCullingName = "cloudAfterCullingName";
    Viewer cloudAfterCulling(cloudAfterCullingName,cloudAfterCullingName);
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud; // use tempCloud for visualization
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> vecTempCloud;
    pcl::PointCloud<pcl::PointXYZRGB> CylinCloud;

    for(int i=0; i< vecCylinderPoints.size();i++) {


        CylinCloud.width = (int) vecCylinderPoints[i].size();
        CylinCloud.height = 1;
        CylinCloud.is_dense = false;
        CylinCloud.points.resize(CylinCloud.width * CylinCloud.height);
        for(int j=0; j< vecCylinderPoints[i].size();++j) {

            CylinCloud.points[j] = vecCylinderPoints[i][j];
            //        tempCloud.points[j].x = vecCylinderPoints[i][j].x;
    //        tempCloud.points[j].y = vecCylinderPoints[i][j].y;
    //        tempCloud.points[j].z = vecCylinderPoints[i][j].z;
    //        cout << vecCylinderPoints[i][j] << endl;

            tempCloud.push_back(vecCylinderPoints[i][j]);

        }

        vecTempCloud.push_back(CylinCloud);

        cout << "finish adding pointcloud to tempCloud" << endl;

    }


    for(int i=0;i< vecTempCloud.size(); i++) {

        std::string eachCloudName = "cloudAfterCullingName"+ std::to_string(i);

//        cout << eachCloudName << endl;

        cout << vecTempCloud[i].points.size() << endl;
    }



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloudPtr(&tempCloud);

    cloudAfterCulling.addPointcloud(tempCloudPtr,cloudAfterCullingName);



    cout << "showing cloud" << endl;
    cloudAfterCulling.run();


    double epsAngle2 = dEpsAngle/180 * M_PI; // 5.0/180 * M_PI;
    bool bFinalCylinderSegmentation = false;


//create vector of all trees
    std::vector<Tree> foundTrees;


    std::vector<pcl::ModelCoefficientsPtr> vecRefinedCylinders;

    std::string viewRefinedCylinderName = "viewRefinedCylinderName";
    std::string testName = "test";
    Viewer viewRefinedCylinder(viewRefinedCylinderName,viewRefinedCylinderName);
    Viewer test(testName,testName);

    cout << "Size of vecTempCloud : " << vecTempCloud.size() << endl;

    for(int i=0; i< vecTempCloud.size();i++) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderCloudPtr2(new pcl::PointCloud<pcl::PointXYZRGB>);


        for(int j=0;j < vecTempCloud[i].points.size();j++) {

            cylinderCloudPtr->points.push_back(vecTempCloud[i][j]);
        }

        if(cylinderCloudPtr->points.empty()) {

            cout << "skip one cylinder" << endl;
            continue;
        }


            /*
        if(i==0 or i==1) {
            bool bSetOpt = true;
            PlaneProcessor removePlane(cylinderCloudPtr,bSetOpt);

            if(removePlane.segment()) {

                cout << "plane segmented." <<endl;

                for(int j=0;j< removePlane.getPlanePointCloud_ptr()->points.size();j++) {

                        cylinderCloudPtr2->points.push_back(removePlane.getNoPlanePointCloud_ptr()->points[j]);

                }
               // cout << "plane coefficient " << removePlane.getPlaneCoefficient() << endl;



            } else {

                cout << "cant find any plane." << endl;
                continue;
            }


            std::string testName2 = "testName245" + std::to_string(i);
            test.addPlane(removePlane.getPlaneCoefficient(),testName2);

        }
*/

        cout << "size of cylinderPtr : " << cylinderCloudPtr->points.size() << endl;
        CylinderProcessor finalCylinSegment(cylinderCloudPtr,unitPlaneVector, epsAngle, bFinalCylinderSegmentation);

//        if(i==0 or i==1)
//            finalCylinSegment.setCylinderPointCloud_ptr(cylinderCloudPtr2);

        finalCylinSegment.segment();

        if(std::isnan(finalCylinSegment.getCylinderCoefficient()->values[6]))
            continue;

        cout << "radius size : " << finalCylinSegment.getCylinderCoefficient()->values[6] << endl;
        cout << "circumference size : " << finalCylinSegment.getCylinderCoefficient()->values[6] * M_PI *2 << endl;

        vecRefinedCylinders.push_back(finalCylinSegment.getCylinderCoefficient());

        //Create tree and add coefficient and point cloud to the tree
        Tree tree;
        tree.cylinderCoef = finalCylinSegment.getCylinderCoefficient();
        tree.trunkPointcloud = finalCylinSegment.getCylinderPointcloud_ptr();
        tree.isTree = true ;
        tree.relatedPlaneCoef = myPlane.getPlaneCoefficient();
        foundTrees.push_back(tree);


    }

//    test.run();
/*
    for(int i=0;i<foundTrees.size();i++) {

        std::string refinedCylinderName = "RefinedCylinder" + std::to_string(i);
        viewRefinedCylinder.addCylinder(foundTrees[i].cylinderCoef,refinedCylinderName);
    }

    std::string planename = "refplane";
    viewRefinedCylinder.addPointcloud(tempCloudPtr,cloudAfterCullingName);
    viewRefinedCylinder.addPointcloud(planePointcloud, planename );
    viewRefinedCylinder.run();

    for(int i=0;i< possibleGroupCylinders.size(); i++) {
        std::string cylinderViewer2Name = "Show possible cylinders";
        Viewer cylinderViewer2(cylinderViewer2Name,cylinderViewer2Name);

        for(int j=0; j< possibleGroupCylinders[i].size();j++) {
            std::string cylinder2Name = "cy." + std::to_string(i) + "-" +std::to_string(j);
            cylinderViewer2.addCylinder(possibleGroupCylinders[i][j].cylinderCoef,cylinder2Name);

        }
        cylinderViewer2.run();
    }
*/



// Identify tree location
    cout << "starting identify tree location" << endl;
    //create point cloud that has 3D points equal to number of cylinder found
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trees3Dpoint (new pcl::PointCloud<pcl::PointXYZRGB>);
    trees3Dpoint->width = vecRefinedCylinders.size();
    trees3Dpoint->height = 1;
    trees3Dpoint->points.resize(trees3Dpoint->width * trees3Dpoint->height);

    /*
    //adding center of cylinder to trees3Dpoint
    for(size_t i = 0; i < trees3Dpoint->points.size(); i++) {

        trees3Dpoint->points[i].x = vecRefinedCylinders[i]->values[0];
        trees3Dpoint->points[i].y = vecRefinedCylinders[i]->values[1];
        trees3Dpoint->points[i].z = vecRefinedCylinders[i]->values[2];

    }

       //project the 3D points to the plane using Inlier Projection

       pcl::PointCloud<pcl::PointXYZRGB>::Ptr trees3Dpoint_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
       pcl::ProjectInliers<pcl::PointXYZRGB> proj;
       proj.setModelType (pcl::SACMODEL_PLANE);
       proj.setInputCloud (trees3Dpoint);
       proj.setModelCoefficients (myPlane.getPlaneCoefficient());
       proj.filter (*trees3Dpoint_projected);


       //Estimate tree location using plantation distance
       pcl::PointXYZRGB firstTree;
       firstTree.x = trees3Dpoint_projected->points[0].x;
       firstTree.y = trees3Dpoint_projected->points[0].y;
       firstTree.z = trees3Dpoint_projected->points[0].z;
       */

    // convert normal vector of the plane to Eigen::Vector

    //Project the center of the cylinder of each tree along its axis vector to the ground (3D plane)
    for( size_t i = 0; i < foundTrees.size(); i++) {

        Eigen::Vector3d plane(foundTrees[i].relatedPlaneCoef->values[0],
                              foundTrees[i].relatedPlaneCoef->values[1],
                              foundTrees[i].relatedPlaneCoef->values[2]);


        Eigen::Vector3d pointOnLine(foundTrees[i].cylinderCoef->values[0],
                                    foundTrees[i].cylinderCoef->values[1],
                                    foundTrees[i].cylinderCoef->values[2]);


        Eigen::Vector3d line(foundTrees[i].cylinderCoef->values[3],
                             foundTrees[i].cylinderCoef->values[4],
                             foundTrees[i].cylinderCoef->values[5]);

        cout << "value of PointOnLine is : " << pointOnLine << endl;

        //calculate dot product between line and plane
        double lineDotPlane = line.dot(plane);
        cout << "value of lineDotPlane is : " << lineDotPlane << endl;
        if(lineDotPlane == 0) { // line and plane does not intersect to each other.
            cout << "lineDotPlane is zero " << endl;
            continue;
        }

        double t = - (plane[0]*pointOnLine[0] + plane[1]*pointOnLine[1] + plane[2]*pointOnLine[2] + myPlane.getPlaneCoefficient()->values[3]) / lineDotPlane;

        cout << "value of t is " << t << endl;
        cout << "value of line is : " << line << endl;
        double tempX = pointOnLine[0] + line[0]*t;
        double tempY = pointOnLine[1] + line[1]*t;
        double tempZ = pointOnLine[2] + line[2]*t;

        cout << "projected temp Value is : " << tempX << " , " << tempY << " , " << tempZ << endl;
        //round value of tempX, tempY, tempZ

        pcl::PointXYZRGB projectedPoint;
        projectedPoint.x = tempX;
        projectedPoint.y = tempY;
        projectedPoint.z = tempZ;

        cout << "projected point is : " << projectedPoint.x << " , " << projectedPoint.y << " , " << projectedPoint.z << endl;
        // Add the projected point to the cloud
       foundTrees[i].projectedPointOnPlane = projectedPoint;
       cout << "value of foundTrees[i].projectedPointOnPlane is : " << foundTrees[i].projectedPointOnPlane << endl;
    }

//    sleep(2);
    //Create pair of <distance from the origin point, Tree>
    std::vector< std::pair<double, int>> vPairDistanceTree;
    std::vector<Tree> vTempTrees;
        //std::vector<Tree> tempAllTrees;

    for(size_t i = 0; i < foundTrees.size(); i ++) {

        Tree tempTree = foundTrees[i];
        double tempDistance = sqrt( pow(tempTree.projectedPointOnPlane.x,2) +
                                    pow(tempTree.projectedPointOnPlane.y,2) +
                                    pow(tempTree.projectedPointOnPlane.z,2) );

        vPairDistanceTree.push_back( std::make_pair(tempDistance,i) );
        vTempTrees.push_back(foundTrees[i]);
    }

    if(vPairDistanceTree.size() != foundTrees.size())
        cerr << "error occurs at pairing distance" << endl;

    //Sort the pair using the distance in ascending order
    sort(vPairDistanceTree.begin(), vPairDistanceTree.end());

    //Clear foundTrees
    foundTrees.clear();

    for(size_t i = 0; i < vPairDistanceTree.size(); i++) {

        Tree tempTree = vTempTrees[vPairDistanceTree[i].second];
        foundTrees.push_back(tempTree); //Add all sorted trees to the vector
    }

// Identify tree location and its number

    //get direction vector from the first and the last keyframe
        //pcl::PointXYZRGB firstKP = keypoint_ptr->points.front();
        //pcl::PointXYZRGB lastKP = keypoint_ptr->points.back();

    Eigen::Vector3d firstKP(keypoint_ptr->points[1].x,
                            keypoint_ptr->points[1].y,
                            keypoint_ptr->points[1].z);

    Eigen::Vector3d lastKP(keypoint_ptr->points.back().x,
                           keypoint_ptr->points.back().y,
                           keypoint_ptr->points.back().z);

    Eigen::Vector3d vecFirstLastKP(lastKP - firstKP);

    //calculate unit vector of vecFirstLastKP
    Eigen::Vector3d unitVecFirstLast = vecFirstLastKP.normalized();
    cout << "unitVector is : " << unitVecFirstLast << endl;

    //use the calculated unit vector to find the tree
    cout << "total trees in foundTrees : " << foundTrees.size() << endl;

    pcl::PointXYZRGB searchPoint;
    pcl::PointXYZRGB firstTree;

    searchPoint.x = foundTrees.front().projectedPointOnPlane.x;
    searchPoint.y = foundTrees.front().projectedPointOnPlane.y;
    searchPoint.z = foundTrees.front().projectedPointOnPlane.z;
    firstTree.x = foundTrees.front().projectedPointOnPlane.x;
    firstTree.y = foundTrees.front().projectedPointOnPlane.y;
    firstTree.z = foundTrees.front().projectedPointOnPlane.z;


    std::vector<Tree> allTrees;
    size_t numberOfFoundTree = foundTrees.size();


    while(foundTrees.size() > 0) {

        //initialize search point
        double tempX = searchPoint.x;
        double tempY = searchPoint.y;
        double tempZ = searchPoint.z;
        cout << "temp 3D points is : " << tempX << " , " << tempY << " , " << tempZ << endl;

        double distanceFromFirstTree = sqrt(pow(firstTree.x - tempX,2) +
                                            pow(firstTree.y - tempY,2) +
                                            pow(firstTree.z - tempZ,2) );

        cout << "distance from first tree is : " << distanceFromFirstTree << endl;


        //find the nearest tree within a radius (planting distance)
        std::vector<std::pair<double, int> > candidateTrees;

        for(size_t i = 0; i < foundTrees.size(); i++) {

            cout << "current point being checked is : " << foundTrees[i].projectedPointOnPlane.x << " , " <<
                                                           foundTrees[i].projectedPointOnPlane.y << " , " <<
                                                           foundTrees[i].projectedPointOnPlane.z << endl;
            //calculate distance of all points relative to the temp points
            double distance = sqrt(pow(tempX - foundTrees[i].projectedPointOnPlane.x,2) +
                                   pow(tempY - foundTrees[i].projectedPointOnPlane.y,2) +
                                   pow(tempZ - foundTrees[i].projectedPointOnPlane.z,2) );

            cout << "distance is : " << distance << endl;

            if(distance < plantDistanceColumn*1.5)
                candidateTrees.push_back(std::make_pair(distance,i));

        }



        if(candidateTrees.empty()) {

            //add blank tree to allTrees
            Tree blankTree;
            blankTree.isTree = false;
            blankTree.height = 0;
            blankTree.radius = 0;
            allTrees.push_back(blankTree);

        } else {

            // get the tree with nearest distance to the center of the search
            sort(candidateTrees.begin(), candidateTrees.end());
            int treeIndex = candidateTrees.front().second;

            allTrees.push_back(foundTrees[treeIndex]);
            
            //remove the added tree from foundTrees
            foundTrees.erase(foundTrees.begin()+treeIndex);

        }

        searchPoint.x = tempX + plantDistanceColumn*unitVecFirstLast[0];
        searchPoint.y = tempY + plantDistanceColumn*unitVecFirstLast[1];
        searchPoint.z = tempZ + plantDistanceColumn*unitVecFirstLast[2];

        cout << "total tree left in the vector : " << foundTrees.size() << endl;
        sleep(1);
    }
    
    cout << "Was looking for " << allTrees.size() << " trees."<<endl;
    sleep(1);


//Finding highest point for each cylinder

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> allCloudInCylinder;
    cout << "size of all tree is : " << allTrees.size() << endl;
    for(size_t i =0; i< allTrees.size(); i++) {

        //if it's not the tree
        if(!allTrees[i].isTree)
            continue;

        pcl::ModelCoefficients::Ptr cylinder_coeff = allTrees[i].cylinderCoef;
        cout << "current cylinder coefficient "
             << cylinder_coeff->values[0] << endl;
        cout << cylinder_coeff->values[1] << endl;
        cout << cylinder_coeff->values[2] << endl;
        cout << cylinder_coeff->values[3] << endl;
        cout << cylinder_coeff->values[4] << endl;
        cout << cylinder_coeff->values[5] << endl;
        cout << cylinder_coeff->values[6] << endl;


        Eigen::Vector3d point1(cylinder_coeff->values[0],
                               cylinder_coeff->values[1],
                               cylinder_coeff->values[2]);

        Eigen::Vector3d point2(point1[0] + cylinder_coeff->values[3]*30,
                               point1[1] + cylinder_coeff->values[4]*30,
                               point1[2] + cylinder_coeff->values[5]*30);

        Eigen::Vector3d point3(point1[0] - cylinder_coeff->values[3]*30,
                               point1[1] - cylinder_coeff->values[4]*30,
                               point1[2] - cylinder_coeff->values[5]*30);

        double constraintRadius = cylinder_coeff->values[6]*300;


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInCylinder(new pcl::PointCloud<pcl::PointXYZRGB>);

        cout << "total points in upward_point_cloud_ptr->points.size() is "
                << upward_point_cloud_ptr->points.size() << endl;
        for(int j=0; j< upward_point_cloud_ptr->points.size(); j++) {

            pcl::PointXYZRGB tempPoint = upward_point_cloud_ptr->points[j];

            Eigen::Vector3d testPoint(tempPoint.x,
                                      tempPoint.y,
                                      tempPoint.z);

            // qp1 = testPoint - point1
            Eigen::Vector3d qp1(testPoint - point1);

            // qp2 = testPoint - point2
            Eigen::Vector3d qp2(testPoint - point2);

            // p2p1 = point2 - point1
            Eigen::Vector3d p2p1(point2 - point1);

            // p1p2 = point1 - point2
            Eigen::Vector3d p1p2(point1 - point2);

            //1. Check if testPoint is between the two circular facets of the cylinder model
            // with (q-p1) dot (p2-p1) >=0 && (q-p2) dot (p1-p2) >=0
            if( qp1.dot(p2p1) < 0 or qp2.dot(p1p2) <0 )
                continue;

            //2. Check if norm(qp1 x p2p1) <= contraintRadius * norm(p2p1)
            Eigen::Vector3d Crossqp1xp2p1( qp1.cross(p2p1) );

            if(Crossqp1xp2p1.norm() > constraintRadius * p2p1.norm() )
                continue;

            cloudInCylinder->points.push_back(tempPoint);

        }
        allCloudInCylinder.push_back(cloudInCylinder);

        cout << "cloud number " << i << " has " << cloudInCylinder->points.size() << " points" << endl;

        // Doing statistical outliers removal
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_in_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (cloudInCylinder);
        sor.setMeanK (20);
        sor.setStddevMulThresh (1.0);
        sor.filter (*filtered_cloud_in_cylinder);

        std::cerr << "Cloud after filtering: " << std::endl;
        std::cerr << *filtered_cloud_in_cylinder << std::endl;
        cout << "total point in the filtered cloud in cylinder : " << filtered_cloud_in_cylinder->points.size() << endl;

        //Finding the highest point in Y direction
        pcl::PointXYZRGB lowestPoint;
        lowestPoint.x = 0;
        lowestPoint.y = 0;
        lowestPoint.z = 0;

        for(int k=0; k < filtered_cloud_in_cylinder->points.size();k++) {

            pcl::PointXYZRGB tempPoint = filtered_cloud_in_cylinder->points[k];

            if (lowestPoint.y > tempPoint.y)
                lowestPoint = tempPoint;

        }
        cout << "The highest point locates at " << lowestPoint << endl;

        double planeA = allTrees[i].relatedPlaneCoef->values[0];
        double planeB = allTrees[i].relatedPlaneCoef->values[1];
        double planeC = allTrees[i].relatedPlaneCoef->values[2];
        double planeD = allTrees[i].relatedPlaneCoef->values[3];

        double distance = fabs(planeA*lowestPoint.x + planeB*lowestPoint.y + planeC*lowestPoint.z + planeD)
        / sqrt(pow(planeA,2) + pow(planeB,2) + pow(planeC,2));

        cout << "tree height is : " << distance << endl;

        allTrees[i].treetopPointcloud = filtered_cloud_in_cylinder;
        allTrees[i].highestPoint = lowestPoint;
        allTrees[i].height = distance;
        allTrees[i].radius = allTrees[i].cylinderCoef->values[6];

        cout << "end of tree number : " << i+1 << endl;
    }



    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto currentTime = oss.str();

    ofstream treeWriter;
    treeWriter.open("tree_measurement"+ currentTime +".csv",ios::app);
    treeWriter << "number of tree, estimated radius, estimated circumference, estimated height, ground truth circumference, ground truth height, circumference error rate, height error rate" << endl;

    //read groundtruth from CSV file
    std::vector<GroundTruth> vGroundTruth;
    readCSV(sPathToGroundTruth, vGroundTruth);

    int writerIterator ;

    //check if the size of vGroundTruth and vAllTrees are not empty
    if(vGroundTruth.empty() || allTrees.empty()) {

        if(vGroundTruth.empty())
            cerr << "vGroundTruth is empty." << endl;

        if(allTrees.empty())
            cerr << "allTrees is empty." << endl;

        return -1;

    }

    //Set number of writerIterator
    if(vGroundTruth.size() < allTrees.size())
        writerIterator = allTrees.size();
    else
        writerIterator = vGroundTruth.size();


    for(int i =0; i < writerIterator; i++) {

        cout << "writing information of tree number " << i+1 << endl;

        double circumferenceError, heightError;


        if(!allTrees[i].isTree or allTrees.size() < i+1 ) {

            circumferenceError = fabs(0 - vGroundTruth[i].circumference)/ vGroundTruth[i].circumference *100.0 ;
            heightError = fabs(0 - vGroundTruth[i].height)/ vGroundTruth[i].height * 100.0;

            treeWriter << i +1
                       << "," << 0
                       << "," << 0
                       << "," << 0
                       << "," << vGroundTruth[i].circumference
                       << "," << vGroundTruth[i].height
                       << "," << circumferenceError
                       << "," << heightError << endl;

        } else if( vGroundTruth.size() < i+1 ) {
            treeWriter << i +1
                       << "," << 0
                       << "," << 0
                       << "," << 0
                       << "," << "N/A"
                       << "," << "N/A"
                       << "," << "N/A"
                       << "," << "N/A" << endl;
        } else{

            treeWriter << i +1
                       << "," << allTrees[i].radius
                       << "," << allTrees[i].radius * 2 * M_PI
                       << "," << allTrees[i].height
                       << "," << vGroundTruth[i].circumference
                       << "," << vGroundTruth[i].height
                       << "," << circumferenceError
                       << "," << heightError << endl;

        }


    }

    double detectErrorRate = fabs(allTrees.size() - vGroundTruth.size())/ vGroundTruth.size() *100 ;
    treeWriter << endl
               << "tree detection error rate" << "," << detectErrorRate << endl;

               treeWriter.close();
    /*
    ofstream treeWriter;
    treeWriter.open("tree_cloud_size"+ currentTime +".csv",ios::app);
    treeWriter << "keyframe, number of point cloud, c0, c1, c2, c3, c4, c5, c6" << endl;




    for(int i =0; i < allCylinders.size(); i++) {

        if(allCylinders[i].cylinderInliers->indices.size() > 0) {
            treeWriter << i
                       << "," << allCylinders[i].cylinderInliers->indices.size() << "," << allCylinders[i].cylinderCoef->values[0]
                       << "," << allCylinders[i].cylinderCoef->values[1] << "," << allCylinders[i].cylinderCoef->values[2]
                       << "," << allCylinders[i].cylinderCoef->values[3] << "," << allCylinders[i].cylinderCoef->values[4]
                       << "," << allCylinders[i].cylinderCoef->values[5] << "," << allCylinders[i].cylinderCoef->values[6]
                       << endl;

        } else {
            treeWriter << i
                       << "," << 0 << "," << 0
                       << "," << 0 << "," << 0
                       << "," << 0 << "," << 0
                       << "," << 0 << "," << 0
                       << endl;

        }

    }


    treeWriter.close();
*/

}

void readInputFile(std::string inputFile, std::string &sDownPointCloud, std::string &sUpPointCloud, std::string &sDownKF,double &searchRadius,
                   double &dEpAngle, double &plantDistanceColumn, double &plantDistanceRow, std::string &sPathToGroundTruth) {


    cv::FileStorage fSettings;
    fSettings.open(inputFile, cv::FileStorage::READ);

    if(!fSettings.isOpened()) {
        cerr << "Cannot open input file from " << inputFile << endl;
        return;
    }

    std::cout << "opening " << inputFile << endl;

    std::cout<<std::endl<<std::endl<<"Parameters: "<<std::endl;

    fSettings["downward_pointcloud"] >> sDownPointCloud;
    std::cout<<"Get downward_pointcloud from "<< sDownPointCloud <<std::endl;

    fSettings["upward_pointcloud"] >> sUpPointCloud;
    std::cout<<"get upward pointcloud from "<< sUpPointCloud <<std::endl;

    fSettings["downward_keyframes"] >> sDownKF;
    std::cout<<"get downward_keyframes from "<< sDownKF <<std::endl;

    fSettings["search_radius"] >> searchRadius;
    std::cout<<"search radius is "<< searchRadius <<std::endl;

    fSettings["angle_to_detect_cylinder"] >> dEpAngle;
    std::cout<<"angle to detect cylinder is "<< dEpAngle <<std::endl;


    std::string sFarmPath;
    fSettings["farm_information"] >> sFarmPath;
    std::cout<<"read farm information from "<< sFarmPath <<std::endl;

    readFarmInformation(sFarmPath, plantDistanceColumn, plantDistanceRow);

    fSettings["ground_truth_file"] >> sPathToGroundTruth;
    std::cout<<"get ground truth file from "<< sPathToGroundTruth <<std::endl;



}

void readFarmInformation(std::string sFarmPath, double &plantDistanceColumn, double &plantDistanceRow) {

    cv::FileStorage fSettings;
    fSettings.open(sFarmPath, cv::FileStorage::READ);

    if(!fSettings.isOpened()) {
        cerr << "Cannot open farm path from " << sFarmPath << endl;
        return;
    }

    fSettings["distance_column"] >> plantDistanceColumn;
    std::cout<<"planting distance column is "<< plantDistanceColumn <<std::endl;

    fSettings["distance_row"] >> plantDistanceRow;
    std::cout<<"planting distance row is "<< plantDistanceRow <<std::endl;


}



bool comparePoint(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
    if (p1.x != p2.x)
        return p1.x > p2.x;
    else if (p1.y != p2.y)
        return  p1.y > p2.y;
    else
        return p1.z > p2.z;
}

bool equalPoint(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
    if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
        return true;
    return false;
}

void tokenize(const std::string &str, std::vector<std::string> &vTokens) {
    int iPos = 0;
    int iTokBeg = 0;
    while (iPos < (int) str.length())
    {
        if (str[iPos] == ',')
        {
            if (iTokBeg < iPos)
            {
                vTokens.push_back(str.substr(iTokBeg, iPos - iTokBeg));
                iTokBeg = iPos + 1;
            }
        }
        iPos++;
    }
    if (iTokBeg < (int) str.length())
        vTokens.push_back(str.substr(iTokBeg));
}

void readCSV(std::string &_pathToCSVfile, std::vector<GroundTruth> &_vGroundTruth) {

    cout << "read CSV data from " << _pathToCSVfile <<"." << endl;
    //skip name of all columns
    ifstream fTimes;
    fTimes.open(_pathToCSVfile.c_str());
    // Skip first line
    std::string s;
    getline(fTimes,s);

    while(!fTimes.eof()) {

        std::string s;
        getline(fTimes,s);
        if(!s.empty()) {

            std::vector<std::string> vTokens;
            tokenize(s, vTokens);

            if(vTokens.size() >= 3) {

                GroundTruth currentGroundTruth;

                currentGroundTruth.numTree = std::stoi(vTokens[0]);
                currentGroundTruth.circumference = std::stod(vTokens[1]);
                currentGroundTruth.height = std::stod(vTokens[2]);

                _vGroundTruth.push_back(currentGroundTruth);

            }

        }
    }




}


void removePointsRightSide(pcl::PointXYZRGB &firstKFPoint, pcl::PointXYZRGB &lastKFPoint, Eigen::Vector3f &planeNormalVector,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud) {

    //create vector from start point to last point = lastPoint - startPoint
    Eigen::Vector3f vectorA(firstKFPoint.x + planeNormalVector[0],
                            firstKFPoint.y + planeNormalVector[1],
                            firstKFPoint.z + planeNormalVector[2]);


    Eigen::Vector3f vectorB(lastKFPoint.x - firstKFPoint.x,
                            lastKFPoint.y - firstKFPoint.y,
                            lastKFPoint.z - firstKFPoint.z);



    //create Vector that points to the left of the guideVector
    Eigen::Vector3f leftVector = vectorA.cross(vectorB); // A x B = C

    for(size_t i = 0; i < inputCloud->points.size(); i++) {

        pcl::PointXYZRGB tempPoint;
        tempPoint.x = inputCloud->points[i].x;
        tempPoint.y = inputCloud->points[i].y;
        tempPoint.z = inputCloud->points[i].z;
        tempPoint.r = inputCloud->points[i].r;
        tempPoint.g = inputCloud->points[i].g;
        tempPoint.b = inputCloud->points[i].b;

        Eigen::Vector3f testVector(tempPoint.x - firstKFPoint.x,
                                   tempPoint.y - firstKFPoint.y,
                                   tempPoint.z - firstKFPoint.z);

        double numerator = leftVector.dot(testVector);
        double denominator = leftVector.norm() * testVector.norm();
        double angle = acos(numerator/denominator) * 180.0 /M_PI;

        if(angle < 90.0)
            outputCloud->points.push_back(tempPoint);

    }

//    cout << "input pointcloud has " << inputCloud->points.size() << " points." << endl;

//    cout << "output pointcloud has " << outputCloud->points.size() << "points." << endl;


}

void calculatePlaneNormalVector(Eigen::Vector4d &planeEquation, Eigen::Vector3d &planeNormalVector){

    planeNormalVector[0] = planeEquation[0];
    planeNormalVector[1] = planeEquation[1];
    planeNormalVector[2] = planeEquation[2];

    planeNormalVector.normalize();

    Eigen::Vector3d yAxis(0.0 , 1.0 , 0.0);

    double numerator = planeNormalVector.dot(yAxis);

    double denominator = planeNormalVector.norm() * yAxis.norm();

    double angle = acos(numerator/denominator) * 180 / M_PI; // return angle between two vectors in degrees

    if(angle <= 90.0) {
        planeNormalVector = planeNormalVector*-1;
    }


}

void projectPointsToPlane(Eigen::Vector4d &planeEquation, Eigen::Vector3d &planeNormalVector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud){


    // Calculate a unit plane normal vector

    planeNormalVector.normalize();

    if(planeNormalVector.norm() > 1.0) {
        cerr << "norm of the unit vector is greater than zero." << endl;
        return;
    }


    for(size_t i = 0; i < inputCloud->points.size(); i++) {

        //calculate  t = -(Ax1 + By1 + Cz1 +D) /(Aa + Bb + Cc)
        // where Ax + By + Cz + D = 0 is a 3D plane model
        // and x = x1 + at , y = y1 + bt , z = z1 + ct are parametric equation of a 3D line

        pcl::PointXYZRGB tempPoint = inputCloud->points[i];

        double A = planeEquation[0];
        double B = planeEquation[1];
        double C = planeEquation[2];
        double D = planeEquation[3];

        double a = planeNormalVector[0];
        double b = planeNormalVector[1];
        double c = planeNormalVector[2];

        double x1 = tempPoint.x;
        double y1 = tempPoint.y;
        double z1 = tempPoint.z;


        double t  = - (A*x1 + B*y1 + C*z1 + D) / (A*a + B*b + C*c);


        pcl::PointXYZRGB projectedPoint; // projected point is in red colour
        projectedPoint.x = x1 + a*t;
        projectedPoint.y = y1 + b*t;
        projectedPoint.z = z1 + c*t;

        outputCloud->points.push_back(projectedPoint);
    }


}

void projectPairPointIndexToPlane(Eigen::Vector4d &planeEquation, Eigen::Vector3d &planeNormalVector, std::vector<std::pair<pcl::PointXYZRGB, int>> &vPairPointIndex,
                                  std::vector<std::pair<pcl::PointXYZRGB, int>> &vProjectedPairPointIndex){


    // Calculate a unit plane normal vector
    planeNormalVector.normalize();

    if(planeNormalVector.norm() > 1.0) {
        cerr << "norm of the unit vector is greater than zero." << endl;
        return;
    }


    for(size_t i = 0; i < vPairPointIndex.size(); i++) {

        //calculate  t = -(Ax1 + By1 + Cz1 +D) /(Aa + Bb + Cc)
        // where Ax + By + Cz + D = 0 is a 3D plane model
        // and x = x1 + at , y = y1 + bt , z = z1 + ct are parametric equation of a 3D line

        pcl::PointXYZRGB tempPoint = vPairPointIndex[i].first;
        int tempPointIndex = vPairPointIndex[i].second;

        double A = planeEquation[0];
        double B = planeEquation[1];
        double C = planeEquation[2];
        double D = planeEquation[3];

        double a = planeNormalVector[0];
        double b = planeNormalVector[1];
        double c = planeNormalVector[2];

        double x1 = tempPoint.x;
        double y1 = tempPoint.y;
        double z1 = tempPoint.z;


        double t  = - (A*x1 + B*y1 + C*z1 + D) / (A*a + B*b + C*c);


        pcl::PointXYZRGB projectedPoint; // projected point is in red colour
        projectedPoint.x = x1 + a*t;
        projectedPoint.y = y1 + b*t;
        projectedPoint.z = z1 + c*t;

        vProjectedPairPointIndex.push_back(std::make_pair(projectedPoint, tempPointIndex));

//        vProjectedPairPointIndex.push_back(std::make_pair(projectedPoint, i));
    }


}


void searchRadius2d(Eigen::Vector4d &planeEquation, Eigen::Vector3d &planeNormalVector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &projectedKeypoints,
                    std::vector<std::pair<pcl::PointXYZRGB,int>> &vProjectedPairPointIndex,
                    double &plantDistanceRow, double &plantDistanceColumn,
                    std::vector<std::pair<pcl::PointXYZRGB,int>> &vCleanedProjectedPairPointIndex) {

    cout << "total number of pointcloud before search radius 2d is : " << vProjectedPairPointIndex.size() << endl;

    //verify search radius. pick the lowest one between row and column planting distance
    double searchRadius = plantDistanceRow * 0.5;


    for(size_t currentKP = 0; currentKP < projectedKeypoints->points.size(); currentKP++) {

        if( currentKP + 1 >= projectedKeypoints->points.size()) //skip when i reaches the last element of projectedKeypoints
            continue;


        Eigen::Vector3d firstKP(projectedKeypoints->points[currentKP].x,
                                projectedKeypoints->points[currentKP].y,
                                projectedKeypoints->points[currentKP].z);


        Eigen::Vector3d secondKP(projectedKeypoints->points[currentKP+1].x,
                                 projectedKeypoints->points[currentKP+1].y,
                                 projectedKeypoints->points[currentKP+1].z);

        Eigen::Vector3d vecKPs(secondKP - firstKP);

        Eigen::Vector3d leftVector = planeNormalVector.cross(vecKPs);
        leftVector.normalize();

        if(currentKP == 0) {
            cout << "leftVector is : " << leftVector << endl;
            cout << "vecKPs is : " << vecKPs << endl;
        }

        //Assume data collector walks exactly at the middle of tree rows.
        //Shift the first KP to the left by plantDistanceRow * 0.5
        Eigen::Vector3d searchCenter(firstKP + leftVector*searchRadius);

        //extract points within radius search
        std::vector<int> vRemovePairs;
        for(size_t currentPair = 0; currentPair < vProjectedPairPointIndex.size(); currentPair++ ) {

            //calculate distance from search center to any point
            Eigen::Vector3d tempPoint(vProjectedPairPointIndex[currentPair].first.x,
                                      vProjectedPairPointIndex[currentPair].first.y,
                                      vProjectedPairPointIndex[currentPair].first.z);

            Eigen::Vector3d center2tempPoint(tempPoint - searchCenter);

            if(center2tempPoint.norm() <= searchRadius) {
                vRemovePairs.push_back(currentPair);
                vCleanedProjectedPairPointIndex.push_back(vProjectedPairPointIndex[currentPair]);

            }



        }

        //removed points being added to the cleaned projected point pair from the vector
        //start to remove them from the last element
        while(!vRemovePairs.empty()) {

            int removeIndex = vRemovePairs.back();
            vProjectedPairPointIndex.erase(vProjectedPairPointIndex.begin() + removeIndex);
            vRemovePairs.pop_back();


        }


    }

    cout << "total number of point cloud after search radius 2d is : " << vCleanedProjectedPairPointIndex.size() << endl;

}

void extractClusterProjectedIndices(std::vector<pcl::PointIndices> &cluster_indices,  std::vector<std::pair<pcl::PointXYZRGB,int>> &vCleanedProjectedPairPointIndex,
                                    std::vector<std::vector<int>> &vClusteredProjectedPointsIndices){



    for(size_t clusterNumber = 0; clusterNumber < cluster_indices.size(); clusterNumber++) {


        std::vector<int> clusterProjectedPointIndices;

        for(size_t pointIndices = 0; pointIndices < cluster_indices[clusterNumber].indices.size(); pointIndices++) {

            int indexNumber = cluster_indices[clusterNumber].indices[pointIndices];

            //the corresponding pair
            int point3dIndex = vCleanedProjectedPairPointIndex[indexNumber].second;
            clusterProjectedPointIndices.push_back(point3dIndex);


        }

        vClusteredProjectedPointsIndices.push_back(clusterProjectedPointIndices);

    }




}

void get3dPointFrom2dIndices(std::vector<std::vector<int>> &vClusteredProjectedPointsIndices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &originalCloud ,
                             std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vPointClouds){


    //Cluster
    for(size_t clusterNumber = 0; clusterNumber < vClusteredProjectedPointsIndices.size(); clusterNumber++) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        double red = rand() % 255;
        double green = rand() % 255;
        double blue = rand() % 255;

        //Extract points for each cluster
        for(size_t pointNumber = 0; pointNumber < vClusteredProjectedPointsIndices[clusterNumber].size(); pointNumber++) {

            int tempIndex = vClusteredProjectedPointsIndices[clusterNumber][pointNumber];

            pcl::PointXYZRGB tempPoint = originalCloud->points[tempIndex];
            tempPoint.r = red;
            tempPoint.g = green;
            tempPoint.b = blue;

            tempPointCloud->points.push_back(tempPoint);

        }

        tempPointCloud->width = tempPointCloud->points.size();
        tempPointCloud->height = 1;

        vPointClouds.push_back(tempPointCloud);

    }



}

void cloudClustering(Eigen::Vector4d &planeEquation, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud,
                     double &plantDistanceRow, double &plantDistanceColumn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
                     std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vOutputClusteredCloud){

    //remove keypoints from pointcloud
    std::vector<int> keypointIndices;
    for(size_t pointNumber = 0; pointNumber < inputCloud->points.size(); pointNumber++) {

        pcl::PointXYZRGB tempPoint = inputCloud->points[pointNumber];

        uint32_t temprgb = (uint32_t) tempPoint.rgb;
        if (temprgb == 16777215)
            keypointIndices.push_back(pointNumber);

    }

    cout << "total keypoints to be removed : " << keypointIndices.size() << " compare to actual KP number : " << keypoints->points.size() << endl;

    cout << "total number of pointcloud before kp removal : " << inputCloud->points.size() << endl;
    while(!keypointIndices.empty()) {

        int removeIndex = keypointIndices.back();
        inputCloud->points.erase(inputCloud->points.begin() + removeIndex);
        keypointIndices.pop_back();

    }
    cout << "total number of pointcloud after kp removal : " << inputCloud->points.size() << endl;



    std::vector<std::pair<pcl::PointXYZRGB, int>> vPairPointIndex;

    //convert input pointcloud to <point,index> pair
    for(size_t i = 0; i < inputCloud->points.size(); i++) {

        pcl::PointXYZRGB tempPoint = inputCloud->points[i];

        vPairPointIndex.push_back(std::make_pair(tempPoint,i));

    }

    //calculate a plane normal vector
    Eigen::Vector3d planeNormalVector;
    calculatePlaneNormalVector(planeEquation, planeNormalVector);


    //project all keypoints to plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectedKeypoints (new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointsToPlane(planeEquation, planeNormalVector, keypoints, projectedKeypoints);

    //project vPairPointIndex to the plane
    std::vector<std::pair<pcl::PointXYZRGB,int>> vProjectedPairPointIndex;
    projectPairPointIndexToPlane(planeEquation, planeNormalVector, vPairPointIndex,vProjectedPairPointIndex);

    //search 2d radius including finding vector to the left of two keypoints
    std::vector<std::pair<pcl::PointXYZRGB,int>> vCleanedProjectedPairPointIndex;

    searchRadius2d(planeEquation, planeNormalVector, projectedKeypoints, vProjectedPairPointIndex,plantDistanceRow,plantDistanceColumn, vCleanedProjectedPairPointIndex);

    //put cleaned points into a point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cleanedProjectedPointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int i = 0; i < vCleanedProjectedPairPointIndex.size(); i ++ ) {

        pcl::PointXYZRGB tempProjectedCleanPoint = vCleanedProjectedPairPointIndex[i].first;
        tempProjectedCleanPoint.r = 255;
        tempProjectedCleanPoint.g = 0;
        tempProjectedCleanPoint.b = 0;

        cleanedProjectedPointcloud->points.push_back(tempProjectedCleanPoint);

    }

    cout << "total points in cleanedProjectedPointcloud : " << cleanedProjectedPointcloud->points.size() << endl;

    //perform Euclidean clustering on the projected point cloud
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cleanedProjectedPointcloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.5); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (300);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cleanedProjectedPointcloud);
    ec.extract (cluster_indices);

    cout << "there are " << cluster_indices.size() << " clusters in the input cloud." << endl;

    //Extract indices from clusters of projected 2d points
    std::vector<std::vector<int>> vClusteredProjectedPointsIndices;
    extractClusterProjectedIndices(cluster_indices, vCleanedProjectedPairPointIndex, vClusteredProjectedPointsIndices);

    //Extract 3D points from vClusteredProjectedPointsIndices
//    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vPointClouds;
    get3dPointFrom2dIndices(vClusteredProjectedPointsIndices, inputCloud, vOutputClusteredCloud);

    std::string viewClusters = "viewClusters";
    Viewer viewClusteredCloud(viewClusters,viewClusters);


    for(int clusterNum = 0; clusterNum < vOutputClusteredCloud.size(); clusterNum++) {
        std::string clouds = "clouds" + std::to_string(clusterNum) ;
        viewClusteredCloud.addPointcloud(vOutputClusteredCloud[clusterNum],clouds);

    }
    viewClusteredCloud.run();







}
