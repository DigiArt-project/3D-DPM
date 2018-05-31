// Written by Fisichella Thomas
// Date 25/05/2018


#include "Mixture.h"

#include <cstdlib>
#include <sys/timeb.h>




using namespace FFLD;
using namespace std;
using namespace Eigen;



int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
//    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    PointCloudPtr finalCloud( new PointCloudT);
    PointCloudPtr cloud1( new PointCloudT);
    PointCloudPtr cloud2( new PointCloudT);
    PointCloudPtr cloud3( new PointCloudT);
    PointCloudPtr cloud4( new PointCloudT);
    PointCloudPtr cloud5( new PointCloudT);
    PointCloudPtr cloudTmp( new PointCloudT);




    if (readPointCloud("/home/ubuntu/3DDataset/3DDPM/chair_normalized/chair_1_normalize.ply", cloudTmp) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();

    transform1.translation() << 1, 2.0, -1.0;

    pcl::transformPointCloud (*cloudTmp, *cloud1, transform1);

    if (readPointCloud("/home/ubuntu/3DDataset/3DDPM/chair_normalized/chair_3_normalize.ply", cloudTmp) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();

    transform2.translation() << 2, 0, 0.0;

    pcl::transformPointCloud (*cloudTmp, *cloud2, transform2);
    if (readPointCloud("/home/ubuntu/3DDataset/3DDPM/chair_normalized/chair_7_normalize.ply", cloudTmp) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    Eigen::Affine3f transform3 = Eigen::Affine3f::Identity();

    transform3.translation() << 0.5, 0, -0.1;

    pcl::transformPointCloud (*cloudTmp, *cloud3, transform3);
    if (readPointCloud("/home/ubuntu/3DDataset/3DDPM/chair_normalized/chair_6_normalize.ply", cloudTmp) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    Eigen::Affine3f transform4 = Eigen::Affine3f::Identity();

    transform4.translation() << 2.1, 2.0, 0.0;

    pcl::transformPointCloud (*cloudTmp, *cloud4, transform4);
    if (readPointCloud("/home/ubuntu/3DDataset/3DDPM/table.pcd", cloudTmp) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    Eigen::Affine3f transform5 = Eigen::Affine3f::Identity();

    transform5.translation() << 2, 2, 2;

    pcl::transformPointCloud (*cloudTmp, *cloud5, transform5);

    *finalCloud += *cloud1;
    *finalCloud += *cloud2;
    *finalCloud += *cloud3;
    *finalCloud += *cloud4;
    *finalCloud += *cloud5;
    *finalCloud += *cloudTmp;


    Viewer view;

    view.addPC( finalCloud);
//    view.displayCubeLine(chairBox, chairResolution, minChair);
//    view.displayCubeLine(tableBox, tableTesolution, minTable);

    pcl::io::savePCDFileASCII ("smallScene5.pcd", *finalCloud);

    view.show();

    return 0;
}

