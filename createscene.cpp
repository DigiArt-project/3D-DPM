
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

class Test{
public:

    pcl::PointXYZ eigenVectorToPointPcl(Eigen::Vector3f pt){
        pcl::PointXYZ point;
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();

        return point;
    }
    pcl::PointXYZ eigenVectorToPointPcl(int x, int y, int z){
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;

        return point;
    }

    void displayCubeLine(Rectangle& rectangle){
        std::cout<<"Displaying..."<<std::endl;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Scene"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem();

        Rectangle rec = rectangle.changeToPclCoordinateSystem();

        cout << "rec pcl : "<< rec << endl;
        cout << "rec pcl diagonal : "<< rec.diagonal() << endl;


//        viewer->addLine(pcl::PointXYZ(0,0,0), pcl::PointXYZ(1,1,1),"ltest");

        cout<<rec.left()<< " / "<<rec.right()<<" / "<<rec.top()<<endl;

        viewer->addLine(eigenVectorToPointPcl(rec.left(), rec.top(), rec.front()), eigenVectorToPointPcl(rec.left(), rec.bottom(), rec.front()),"l1");
        viewer->addLine(eigenVectorToPointPcl(rec.left(), rec.bottom(), rec.front()), eigenVectorToPointPcl(rec.right(), rec.bottom(), rec.front()),"l2");
        viewer->addLine(eigenVectorToPointPcl(rec.right(), rec.bottom(), rec.front()), eigenVectorToPointPcl(rec.right(), rec.top(), rec.front()),"l3");
        viewer->addLine(eigenVectorToPointPcl(rec.right(), rec.top(), rec.front()), eigenVectorToPointPcl(rec.left(), rec.top(), rec.front()),"l4");


        viewer->addLine(eigenVectorToPointPcl(rec.left(), rec.top(), rec.back()), eigenVectorToPointPcl(rec.left(), rec.bottom(), rec.back()),"l5");
        viewer->addLine(eigenVectorToPointPcl(rec.left(), rec.bottom(), rec.back()), eigenVectorToPointPcl(rec.right(), rec.bottom(), rec.back()),"l6");
        viewer->addLine(eigenVectorToPointPcl(rec.right(), rec.bottom(), rec.back()), eigenVectorToPointPcl(rec.right(), rec.top(), rec.back()),"l7");
        viewer->addLine(eigenVectorToPointPcl(rec.right(), rec.top(), rec.back()), eigenVectorToPointPcl(rec.left(), rec.top(), rec.back()),"l8");

        viewer->addLine(eigenVectorToPointPcl(rec.left(), rec.top(), rec.front()), eigenVectorToPointPcl(rec.left(), rec.top(), rec.back()),"l9");
        viewer->addLine(eigenVectorToPointPcl(rec.left(), rec.bottom(), rec.front()), eigenVectorToPointPcl(rec.left(), rec.bottom(), rec.back()),"l10");
        viewer->addLine(eigenVectorToPointPcl(rec.right(), rec.bottom(), rec.front()), eigenVectorToPointPcl(rec.right(), rec.bottom(), rec.back()),"l11");
        viewer->addLine(eigenVectorToPointPcl(rec.right(), rec.top(), rec.front()), eigenVectorToPointPcl(rec.right(), rec.top(), rec.back()),"l12");


        while (!viewer->wasStopped())
            //while (!model_viewer->wasStopped())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }


};


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
//    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    PointCloudPtr cloud1( new PointCloudT);
    PointCloudPtr cloud2( new PointCloudT);

    if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/3DDataset/3DDPM/chair.pcd", *cloud1) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/3DDataset/3DDPM/table.pcd", *cloud2) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }


    PointType min;
    PointType max;
    pcl::getMinMax3D(*cloud1 , min, max);


    Rectangle rec(Eigen::Vector3i(min.x, min.y, min.z), max.x, max.y, max.z);
    Test test;
    test.displayCubeLine(rec);

    return 0;
}

