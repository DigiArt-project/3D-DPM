
#include "Mixture.h"

#include <cstdlib>
#include <sys/timeb.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;

class Viewer{
public:

    Viewer() : viewer (new pcl::visualization::PCLVisualizer ("Scene")), id(0){

        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem();
    }

    void addPC( PointCloudPtr cloud, Eigen::Vector3i color = Eigen::Vector3i(0, 255, 0)){
        pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, color(0), color(1), color(2));
        string name = "cloud";
        name.append(std::to_string(id));
        viewer->addPointCloud( cloud, single_color, name.c_str(), 0);
        ++id;
    }

    void displayCubeLine(Rectangle& rectangle, float resolution, PointType offset,
                         Eigen::Vector3i color = Eigen::Vector3i(255, 0, 0), string name = ""){

        Rectangle rec = rectangle.changeToPclCoordinateSystem();

        cout << "rec pcl : "<< rec << endl;
        cout << "rec pcl diagonal : "<< rec.diagonal() << endl;
        float left = rec.left()*resolution+offset.x;
        float top = rec.top()*resolution+offset.y;
        float right = rec.right()*resolution+offset.x;
        float front = rec.front()*resolution+offset.z;
        float bottom = rec.bottom()*resolution+offset.y;
        float back = rec.back()*resolution+offset.z;

        cout << "line left : "<< left << endl;
        cout << "line right : "<< right << endl;
        cout << "line top : "<< top << endl;
        cout << "line bottom : "<< bottom << endl;
        cout << "line front : "<< front << endl;
        cout << "line back : "<< back << endl;
        viewer->addLine(pcl::PointXYZ(left, top, front), pcl::PointXYZ(left, bottom, front), color(0), color(1), color(2), name.append(std::to_string(id)).append(":l1"));
        viewer->addLine(pcl::PointXYZ(left, bottom, front), pcl::PointXYZ(right, bottom, front),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l2"));
        viewer->addLine(pcl::PointXYZ(right, bottom, front), pcl::PointXYZ(right, top, front),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l3"));
        viewer->addLine(pcl::PointXYZ(right, top, front), pcl::PointXYZ(left, top, front),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l4"));


        viewer->addLine(pcl::PointXYZ(left, top, back), pcl::PointXYZ(left, bottom, back),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l5"));
        viewer->addLine(pcl::PointXYZ(left, bottom, back), pcl::PointXYZ(right, bottom, back),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l6"));
        viewer->addLine(pcl::PointXYZ(right, bottom, back), pcl::PointXYZ(right, top, back),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l7"));
        viewer->addLine(pcl::PointXYZ(right, top, back), pcl::PointXYZ(left, top, back),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l8"));

        viewer->addLine(pcl::PointXYZ(left, top, front), pcl::PointXYZ(left, top, back),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l9"));
        viewer->addLine(pcl::PointXYZ(left, bottom, front), pcl::PointXYZ(left, bottom, back),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l10"));
        viewer->addLine(pcl::PointXYZ(right, bottom, front), pcl::PointXYZ(right, bottom, back),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l11"));
        viewer->addLine(pcl::PointXYZ(right, top, front), pcl::PointXYZ(right, top, back),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l12"));

        ++id;
    }

    void show(){
        while (!viewer->wasStopped())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    int id;
};


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
//    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    PointCloudPtr cloud( new PointCloudT);
    PointCloudPtr cloud1( new PointCloudT);
    PointCloudPtr cloud2( new PointCloudT);
    PointCloudPtr tmpCloud( new PointCloudT);

    if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/3DDataset/3DDPM/chair.pcd", *cloud1) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/3DDataset/3DDPM/table.pcd", *cloud2) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform.translation() << 0,0,0;
//    transform.translation() << 2,2,2;
    pcl::transformPointCloud (*cloud1, *cloud, transform);

    PointType min;
    PointType max;
    pcl::getMinMax3D(*cloud , min, max);


    tmpCloud->points.resize(2);
    tmpCloud->width = 2;
    tmpCloud->height = 1;
    tmpCloud->points[0]=min;
    tmpCloud->points[1]=max;
    cout<<"test::min : "<<min<<endl;
    cout<<"test::max : "<<max<<endl;

    float resolution = GSHOTPyramid::computeCloudResolution(cloud1);
    Model::triple<int, int, int> sceneSize( (max.z-min.z)/resolution+1, (max.y-min.y)/resolution+1, (max.x-min.x)/resolution+1);

    Rectangle rec(Eigen::Vector3i(0,0,0), sceneSize.third, sceneSize.second, sceneSize.first);
    Viewer view;

    view.addPC( cloud);
    view.addPC( tmpCloud);
    view.displayCubeLine(rec, resolution, min);


//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(tmpCloud, 255, 0, 0);
//    view.viewer->addPointCloud( tmpCloud, single_color2, "cloudtmp", 0);



    view.show();





    return 0;
}

