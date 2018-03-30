
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

    if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/3DDataset/3DDPM/chair.pcd", *cloud1) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/3DDataset/3DDPM/table.pcd", *cloud2) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform.translation() << 1, 0.0, 0.0;

    pcl::transformPointCloud (*cloud1, *finalCloud, transform);

    PointType minChair;
    PointType maxChair;
    pcl::getMinMax3D(*finalCloud , minChair, maxChair);

//    cout<<"test::min : "<<minChair<<endl;
//    cout<<"test::max : "<<maxChair<<endl;

    float chairResolution = GSHOTPyramid::computeCloudResolution(finalCloud);
    Model::triple<int, int, int> chairSize( (maxChair.z-minChair.z)/chairResolution+1,
                                            (maxChair.y-minChair.y)/chairResolution+1,
                                            (maxChair.x-minChair.x)/chairResolution+1);

    Rectangle chairBox(Eigen::Vector3i(0,0,0), chairSize.third, chairSize.second, chairSize.first);


    PointType minTable;
    PointType maxTable;
    pcl::getMinMax3D(*cloud2 , minTable, maxTable);


    float tableTesolution = GSHOTPyramid::computeCloudResolution(cloud2);
    Model::triple<int, int, int> tableSize( (maxTable.z-minTable.z)/tableTesolution+1,
                                            (maxTable.y-minTable.y)/tableTesolution+1,
                                            (maxTable.x-minTable.x)/tableTesolution+1);

    Rectangle tableBox(Eigen::Vector3i(0,0,0), tableSize.third, tableSize.second, tableSize.first);




    *finalCloud += *cloud2;

    Viewer view;

    view.addPC( finalCloud);
    view.displayCubeLine(chairBox, chairResolution, minChair);
    view.displayCubeLine(tableBox, tableTesolution, minTable);

    pcl::io::savePCDFileASCII ("scene.pcd", *finalCloud);

    view.show();

    return 0;
}

