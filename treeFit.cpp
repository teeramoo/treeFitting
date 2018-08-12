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

class Tree
{
public:
    pcl::ModelCoefficients::Ptr cylinderCoef;
    pcl::ModelCoefficients::Ptr relatedPlaneCoef;

    pcl::PointCloud<pcl::PointXYZRGB> trunkPointcloud;
    pcl::PointCloud<pcl::PointXYZRGB> treetopPointcloud;


    bool isTree;
    double radius;
    double height;
    Eigen::Vector3d projectedPointOnPlane;
    pcl::PointXYZRGB highestPoint;


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
   

    if(argc != 7) {
        cout << "Usage : " << argv[0] << "<Pointcloud from downward camera> <Pointcloud from upward camera> "
                                         "<3D points representing keyframe> <search radius> <angle to detect cylinder> <planting distance>" << endl;
        return -1;
    }

    //Get inputs
    std::string sInput = std::string(argv[1]);
    std::string sUpwardInput = std::string(argv[2]);
    std::string sInputKeyframe = std::string(argv[3]);
    double dPointDistance = std::stod(argv[4]);
    double dEpsAngle = std::stod(argv[5]);
    double plantDistance = std::stod(argv[6]);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downward_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoint_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr upward_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Read input clouds
    //Input cloud will have pcd format and color in float bits (reinterpret cast from uint32_t)
    pcl::io::loadPCDFile(sInput, *downward_point_cloud_ptr);
    pcl::io::loadPCDFile(sInputKeyframe, *keypoint_ptr);
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

    if (!myPlane.segment())
        return -1;

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


    NormalEstimator planeNormals(myPlane.getNoPlanePointCloud_ptr(), 5.5);
    planeNormals.calculate();

    cout << "calculate plane normal" << endl;

    std::string noPlaneNormalName = "no plane normals";
    Viewer noPlaneNormalViewer(noPlaneNormalName, noPlaneNormalName);
    std::string noPlanePointCloudName = "no plane PC";
    noPlaneNormalViewer.addPointcloud(planeNormals.getInputPointCloud(), noPlanePointCloudName);
    noPlaneNormalViewer.addNormals(planeNormals.getInputPointCloud(), planeNormals.getCloudNormal(), 10, 0.05,
                                   noPlaneNormalName);

    noPlaneNormalViewer.run();


    std::vector<Cylinder> allCylinders;

    double epsAngle = dEpsAngle/180*M_PI;
    bool bInitCylinderOptimization = false;
    Eigen::Vector3f planeVector = myPlane.getPlaneVector();


    // Point Cloud filter using keypoints
    for (int i = 0; i < keypoint_ptr->points.size(); i++) {


        pcl::PointXYZRGB tempKeyPoint = keypoint_ptr->points[i];
        double tkpx = tempKeyPoint.x;
        double tkpy = tempKeyPoint.y;
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
    }

    //End of Point Cloud filter




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
    double epsAngle2 = 5/180 * M_PI;
    bool bFinalCylinderSegmentation = false;

    std::vector<pcl::ModelCoefficientsPtr> vecRefinedCylinders;

    std::string viewRefinedCylinderName = "viewRefinedCylinderName";
    std::string testName = "test";
    Viewer viewRefinedCylinder(viewRefinedCylinderName,viewRefinedCylinderName);
    Viewer test(testName,testName);


    for(int i=0; i< vecTempCloud.size();i++) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderCloudPtr2(new pcl::PointCloud<pcl::PointXYZRGB>);


        for(int j=0;j < vecTempCloud[i].points.size();j++) {

            cylinderCloudPtr->points.push_back(vecTempCloud[i][j]);
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


        CylinderProcessor finalCylinSegment(cylinderCloudPtr,planeVector, epsAngle, bFinalCylinderSegmentation);

//        if(i==0 or i==1)
//            finalCylinSegment.setCylinderPointCloud_ptr(cylinderCloudPtr2);

        finalCylinSegment.segment();

        cout << "radius size : " << finalCylinSegment.getCylinderCoefficient()->values[6] << endl;
        cout << "circumference size : " << finalCylinSegment.getCylinderCoefficient()->values[6] * M_PI *2 << endl;

        vecRefinedCylinders.push_back(finalCylinSegment.getCylinderCoefficient());
    }

    test.run();

    for(int i=0;i<vecRefinedCylinders.size();i++) {

        std::string refinedCylinderName = "RefinedCylinder" + std::to_string(i);
        viewRefinedCylinder.addCylinder(vecRefinedCylinders[i],refinedCylinderName);
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
   //     cylinderViewer2.run();
    }

//create vector of all trees
    std::vector<Tree> allTrees;
    for(size_t i = 0 ; i < vecRefinedCylinders.size(); i++) {

        Tree tree;
        tree.isTree = true;
        tree.cylinderCoef = vecRefinedCylinders[i];
        tree.relatedPlaneCoef = myPlane.getPlaneCoefficient();
        tree.trunkPointcloud = vecRefinedCylinders[i];

    }


// Identify tree location

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
    Eigen::Vector3d plane(myPlane.getPlaneCoefficient()->values[0],
                          myPlane.getPlaneCoefficient()->values[1],
                          myPlane.getPlaneCoefficient()->values[2]);

    //Project the center of the cylinder of each tree along its axis vector to the ground (3D plane)
    for( size_t i = 0; i < trees3Dpoint->points.size(); i++) {

        Eigen::Vector3d pointOnLine(vecRefinedCylinders[i]->values[0],
                                    vecRefinedCylinders[i]->values[1],
                                    vecRefinedCylinders[i]->values[2]);


        Eigen::Vector3d line(vecRefinedCylinders[i]->values[3],
                             vecRefinedCylinders[i]->values[4],
                             vecRefinedCylinders[i]->values[5]);



        //calculate dot product between line and plane
        double lineDotPlane = line.dot(plane);

        if(lineDotPlane == 0) { // line and plane does not intersect to each other.
            cout << "lineDotPlane is zero " << endl;
            continue;
        }

        double t = - (plane[0]*pointOnLine[0] + plane[1]*pointOnLine[1] + plane[2]*pointOnLine[2] + myPlane.getPlaneCoefficient()->values[3]) / lineDotPlane;

        double tempX = pointOnLine[0] + line[0]*t;
        double tempY = pointOnLine[1] + line[1]*t;
        double tempZ = pointOnLine[2] + line[2]*t;

        pcl::PointXYZRGB projectedPoint(tempX,tempY,tempZ);

        // Add the projected point to the cloud
        trees3Dpoint->points.push_back(projectedPoint);
    }

    //Sort the center of each tree with distance from the origin point to the center
    std::vector< std::pair<pcl::PointXYZRGB, double>> pairPointDistance;



//Finding highest point for each cylinder

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> allCloudInCylinder;

    for(int i =0; i< vecRefinedCylinders.size(); i++) {

        pcl::ModelCoefficients::Ptr cylinder_coeff = vecRefinedCylinders[i];
        cout << "vecCylinder coefficient " <<
             vecRefinedCylinders[i]->values[0] << endl;
        cout << vecRefinedCylinders[i]->values[1] << endl;
        cout << vecRefinedCylinders[i]->values[2] << endl;
        cout << vecRefinedCylinders[i]->values[3] << endl;
        cout << vecRefinedCylinders[i]->values[4] << endl;
        cout << vecRefinedCylinders[i]->values[5] << endl;
        cout << vecRefinedCylinders[i]->values[6] << endl;


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
        cout << "The lowest point locates at " << lowestPoint << endl;

        double planeA = myPlane.getPlaneCoefficient()->values[0];
        double planeB = myPlane.getPlaneCoefficient()->values[1];
        double planeC = myPlane.getPlaneCoefficient()->values[2];
        double planeD = myPlane.getPlaneCoefficient()->values[3];

        double distance = fabs(planeA*lowestPoint.x + planeB*lowestPoint.y + planeC*lowestPoint.z + planeD)
        / sqrt(pow(planeA,2) + pow(planeB,2) + pow(planeC,2));

        cout << "tree height is : " << distance << endl;
    }



    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto currentTime = oss.str();

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


}


