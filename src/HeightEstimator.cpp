//
// Created by teeramoo on 2/8/2561.
//

#include "HeightEstimator.h"

HeightEstimator::HeightEstimator(pcl::ModelCoefficientsPtr &_cylinderCoefficient) {

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