#ifndef VIEWER
#define VIEWER

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

#include "Rectangle.h"
namespace FFLD
{
class Viewer{
public:

    Viewer() : viewer (new pcl::visualization::PCLVisualizer ("Scene")), id(0){

        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem();
    }

    void addPC( PointCloudPtr cloud, int ptSize = 3, Eigen::Vector3i color = Eigen::Vector3i(0, 255, 0)){
        pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, color(0), color(1), color(2));
        string name = "cloud";
        name.append(std::to_string(id));
        viewer->addPointCloud( cloud, /*single_color,*/ name.c_str(), 0);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ptSize, name.c_str());
        ++id;
    }

    void displayCubeLine(Rectangle& rectangle, Eigen::Vector3f offset = Eigen::Vector3f(0, 0, 0),
                         Eigen::Vector3i color = Eigen::Vector3i(255, 0, 0), string name = ""){

        Rectangle rec = rectangle.changeToPclCoordinateSystem();

//        cout << "rec pcl : "<< rec << endl;
//        cout << "rec pcl diagonal : "<< rec.diagonal() << endl;
        float left = rec.left()*rec.resolution() + offset(0);
        float top = rec.top()*rec.resolution() + offset(1);
        float right = rec.right()*rec.resolution() + offset(0);
        float front = rec.front()*rec.resolution() + offset(2);
        float bottom = rec.bottom()*rec.resolution() + offset(1);
        float back = rec.back()*rec.resolution() + offset(2);

//        cout << "line left : "<< left << endl;
//        cout << "line right : "<< right << endl;
//        cout << "line top : "<< top << endl;
//        cout << "line bottom : "<< bottom << endl;
//        cout << "line front : "<< front << endl;
//        cout << "line back : "<< back << endl;
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

}
#endif // VIEWER

