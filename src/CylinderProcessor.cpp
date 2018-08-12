//
// Created by teeramoo on 25/4/2561.
//

#include "CylinderProcessor.h"

CylinderProcessor::~CylinderProcessor() {
//    cout << "Cylinder segmentation has been destroyed." << endl;
}

CylinderProcessor::CylinderProcessor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, Eigen::Vector3f &normalVector,
                                 double &epsAngle, bool &bOptimization) {

    cylinderSegmenter.setOptimizeCoefficients(bOptimization);
    cylinderSegmenter.setModelType (pcl::SACMODEL_CYLINDER);
    cylinderSegmenter.setAxis(normalVector);
    cylinderSegmenter.setEpsAngle(epsAngle);
    cylinderSegmenter.setMethodType (pcl::SAC_RANSAC);
    cylinderSegmenter.setNormalDistanceWeight (0.1); // 0.0
    cylinderSegmenter.setMaxIterations (50000);
    cylinderSegmenter.setDistanceThreshold (0.6); //0.2
    cylinderSegmenter.setRadiusLimits (0.01, 0.1);
    cylinderSegmenter.setInputCloud (inputPointCloud);

    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderPointCloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    setCylinderInliers(inliers_cylinder);
    setCylinderCoefficient(coefficients_cylinder);
    setCylinderPointCloud_ptr(cylinderPointCloud_ptr);

    NormalEstimator normalEstimator(inputPointCloud,1.0);
    normalEstimator.calculate();
    cylinderSegmenter.setInputNormals (normalEstimator.getCloudNormal());
}

CylinderProcessor::CylinderProcessor(double &_distanceLimit, enum CylinderProcessor::refineOptions _refineOption) {


    distanceLimit = _distanceLimit;
    refineOption = _refineOption;

}

void CylinderProcessor::cluster(std::vector<Cylinder> &allCylinders,
                                std::vector<std::vector<Cylinder>> &possibleGroupCylinders ) {

    std::vector<Cylinder> possibleCylinders;

    double prevC0 = 0.0, prevC1=0.0, prevC2=0.0, prevC3=0.0, prevC4=0.0, prevC5=0.0, prevC6=0.0;


    for(int i = 0; i < allCylinders.size(); i++) {

        double curC0, curC1, curC2, curC3, curC4, curC5, curC6;

        //Check number of point cloud
        if(!allCylinders[i].cylinderInliers->indices.empty()) {

            curC0 = allCylinders[i].cylinderCoef->values[0];
            curC1 = allCylinders[i].cylinderCoef->values[1];
            curC2 = allCylinders[i].cylinderCoef->values[2];
            curC3 = allCylinders[i].cylinderCoef->values[3];
            curC4 = allCylinders[i].cylinderCoef->values[4];
            curC5 = allCylinders[i].cylinderCoef->values[5];
            curC6 = allCylinders[i].cylinderCoef->values[6];
        }
        else {
            curC0 = 0.0;
            curC1 = 0.0;
            curC2 = 0.0;
            curC3 = 0.0;
            curC4 = 0.0;
            curC5 = 0.0;
            curC6 = 0.0;
        }

        double distance;

        if(refineOption == XYZ)
            distance = sqrt(pow(curC0-prevC0,2)+pow(curC1-prevC1,2)+pow(curC2-prevC2,2));
        else if(refineOption == XZ)
            distance = sqrt(pow(curC0-prevC0,2)+pow(curC2-prevC2,2));
        else {
            cout << "unknown refine option." << endl;
            return;
        }

        if(distance > distanceLimit or i == allCylinders.size()-1) {


            if(possibleCylinders[0].cylinderInliers->indices.empty()) {

                possibleCylinders.clear();
                cout << "Delete a group of non cylinder" << endl;

            }
            else {

                possibleGroupCylinders.push_back(possibleCylinders);
                possibleCylinders.clear();
                cout << "Group number " << possibleGroupCylinders.size() << " has " << possibleGroupCylinders.back().size() << " cylinders." << endl;
            }


        }
        else
        {
            possibleCylinders.push_back(allCylinders[i]);

        }

        prevC0 = curC0;
        prevC1 = curC1;
        prevC2 = curC2;
        prevC3 = curC3;
        prevC4 = curC4;
        prevC5 = curC5;
        prevC6 = curC6;


    }


}

void CylinderProcessor::segment() {
    cout << "before segment " << endl;
    cylinderSegmenter.segment(*cylinderInliers,*cylinderCoefficients);
    cout << "after segment " << endl;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract (true);
    extract.setInputCloud (cylinderSegmenter.getInputCloud());
    extract.setIndices (cylinderInliers);
    extract.setNegative (false);
    extract.filter(*cylinderPointcloud_ptr);

}

void CylinderProcessor::segment(pcl::PointXYZRGB &refKeyframe) {
//    cout << "about to segment a cylinder" << endl;
    cylinderSegmenter.segment(*cylinderInliers,*cylinderCoefficients);
//    cout << "done segmenting a cylinder" << endl;

    if(cylinderCoefficients->values.size() == 0 or cylinderInliers->indices.size() < 20 ) {

        cylinderCoefficients->header.frame_id = "SKIP";
        cylinderPointcloud_ptr = NULL;
        cylinderInliers->indices.erase(cylinderInliers->indices.begin(),cylinderInliers->indices.end());

        if(!cylinderInliers->indices.empty()) {
            cout << "there are some points in the cylinder Inliers. Terminated." << endl;
            return;

        }

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

//        cout << "skip adding cylinder due to too far distance " << endl;
        return;
    }

//    cout << "Cylinder coefficient is : " << *cylinderCoefficients << endl;

    //Extract point cloud representing the cylinder
    pcl::ExtractIndices<pcl::PointXYZRGB> extract (true);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCylinderCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    extract.setInputCloud (cylinderSegmenter.getInputCloud());
    extract.setIndices (cylinderInliers);
    extract.setNegative (false);
    extract.filter(*cylinderPointcloud_ptr);

//    cout << "Size of cylinderPointcloud_ptr is : " << cylinderPointcloud_ptr->points.size() << endl;

}

void CylinderProcessor::removeDuplicates(std::vector<std::vector<Cylinder>> &possibleGroupCylinders, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &allPointsAfterCulling) {

    std::vector<std::vector<pcl::PointXYZRGB>> vecCylinderPoints;
    //Recalculate point cloud for a cylinder

    if(!cloud2vec(possibleGroupCylinders,vecCylinderPoints))
        return;

    cout << "check size of vecCylinderPoints : " << vecCylinderPoints.size() << endl;


    for(int i=0;i< vecCylinderPoints.size();i++) {

        sortPoints(vecCylinderPoints[i]);

        int uniqueEndIndex;
        equalPoint(vecCylinderPoints[i], uniqueEndIndex);

      //  auto unique_end = std::unique(vecCylinderPoints[i].begin(), vecCylinderPoints[i].end(), equalPoint);

        //Sorting problem --> Try displaying all of them vs points after culling
        cout << "Number of points before culling : " << vecCylinderPoints[i].size() << endl;

        cout << "uniqueEndIndex is : " << uniqueEndIndex << endl;
        vecCylinderPoints[i].erase(vecCylinderPoints[i].begin()+uniqueEndIndex, vecCylinderPoints[i].end());

        cout << "Number of points after culling : " << vecCylinderPoints[i].size() << endl;


        pcl::PointCloud<pcl::PointXYZRGB> pointCloudCylinderAfterCulling;
        for(int j=0;j<vecCylinderPoints[0].size();j++) {

            pointCloudCylinderAfterCulling.push_back(vecCylinderPoints[i][j]);
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderPoints_ptrAfterCulling(&pointCloudCylinderAfterCulling);
        allPointsAfterCulling.push_back(cylinderPoints_ptrAfterCulling);

    }


}


bool CylinderProcessor::cloud2vec(std::vector<std::vector<Cylinder>> &possibleGroupCylinders,
                                  std::vector<std::vector<pcl::PointXYZRGB>> &vecCylinderPoints) {

    for(int i=0;i<possibleGroupCylinders.size();i++) {

        std::vector<pcl::PointXYZRGB> cylinderPoints;
        int checkNumPoints = 0;



        for (int j = 0; j < possibleGroupCylinders[i].size(); j++) {


            for (int k = 0; k < possibleGroupCylinders[i][j].cylinderPointCloud_ptr->points.size(); k++) {

                cylinderPoints.push_back(possibleGroupCylinders[i][j].cylinderPointCloud_ptr->points[k]);
                checkNumPoints++;
            }


            cout << "checkNumPoints / cylinderPoints.size() is : " << checkNumPoints << " / " << cylinderPoints.size()
                 << endl;
            if (checkNumPoints != cylinderPoints.size()) {

                cout << "something wrong at cylinder recalculation" << endl;

                return false;
            }



        }


        vecCylinderPoints.push_back(cylinderPoints);


    }

    return true;

}

void CylinderProcessor::sortPoints(std::vector<pcl::PointXYZRGB> vecPoints) {

    sortStruct sorter(this);
    std::sort(vecPoints.begin(),vecPoints.end(), sorter);

}

void CylinderProcessor::equalPoint(std::vector<pcl::PointXYZRGB> vecSortedPoints, int &uniqueEndIndex) {

    equalPointStruct equalChecker(this);
    auto unique_end = std::unique(vecSortedPoints.begin(), vecSortedPoints.end(), equalChecker);

    uniqueEndIndex = std::distance(vecSortedPoints.begin(),unique_end);

}


/*
bool CylinderProcessor::comparePoint(const pcl::PointXYZRGB p1, const pcl::PointXYZRGB p2) {

    if (p1.x != p2.x)
        return p1.x > p2.x;
    else if (p1.y != p2.y)
        return  p1.y > p2.y;
    else
        return p1.z > p2.z;

}

bool CylinderProcessor::equalPoint(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {

    if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
        return true;
    return false;

}

 */

void CylinderProcessor::setCylinderCoefficient(pcl::ModelCoefficients::Ptr &_cylinderCoefficient) {
    cylinderCoefficients = _cylinderCoefficient;
}

pcl::ModelCoefficients::Ptr CylinderProcessor::getCylinderCoefficient() {
    return cylinderCoefficients;
}

void CylinderProcessor::setCylinderInliers(pcl::PointIndices::Ptr &_cylinderInliers) {
    cylinderInliers = _cylinderInliers;
}

pcl::PointIndices::Ptr CylinderProcessor::getCylinderInliers() {
    return cylinderInliers;
}

void CylinderProcessor::setCylinderPointCloud_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cylinderPointCloud_ptr) {
    cylinderPointcloud_ptr = _cylinderPointCloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CylinderProcessor::getCylinderPointcloud_ptr() {
    return cylinderPointcloud_ptr;
}
