// Written by Fisichella Thomas
// Date 25/05/2018

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
        viewer->addPointCloud( cloud, single_color, name.c_str(), 0);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ptSize, name.c_str());
        ++id;
    }

    void displayCubeLine(const Rectangle& rec, Eigen::Vector3i color = Eigen::Vector3i(255, 0, 0), string name = ""){

        viewer->addLine(rec.cloud(0), rec.cloud(1),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l1"));
        viewer->addLine(rec.cloud(2), rec.cloud(3),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l2"));
        viewer->addLine(rec.cloud(4), rec.cloud(5),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l3"));
        viewer->addLine(rec.cloud(6), rec.cloud(7),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l4"));


        viewer->addLine(rec.cloud(0), rec.cloud(2),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l5"));
        viewer->addLine(rec.cloud(1), rec.cloud(3),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l6"));
        viewer->addLine(rec.cloud(4), rec.cloud(6),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l7"));
        viewer->addLine(rec.cloud(5), rec.cloud(7),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l8"));

        viewer->addLine(rec.cloud(0), rec.cloud(4),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l9"));
        viewer->addLine(rec.cloud(1), rec.cloud(5),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l10"));
        viewer->addLine(rec.cloud(2), rec.cloud(6),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l11"));
        viewer->addLine(rec.cloud(3), rec.cloud(7),color(0), color(1), color(2), name.append(std::to_string(id)).append(":l12"));

        ++id;
    }

    void displayAxis( float* rf, pcl::PointXYZ origin = pcl::PointXYZ(0,0,0), float ratio = 1, int ptSize = 1){
        viewer->addLine(origin, pcl::PointXYZ(origin.x+rf[0]/ratio,
                                              origin.y+rf[1]/ratio,
                                              origin.z+rf[2]/ratio), 255,0,0, std::to_string(id).append(":l1"));
        viewer->addLine(origin, pcl::PointXYZ(origin.x+rf[3]/ratio,
                                              origin.y+rf[4]/ratio,
                                              origin.z+rf[5]/ratio), 0,255,0, std::to_string(id).append(":l2"));
        viewer->addLine(origin, pcl::PointXYZ(origin.x+rf[6]/ratio,
                                              origin.y+rf[7]/ratio,
                                              origin.z+rf[8]/ratio), 0,0,255, std::to_string(id).append(":l3"));
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH , ptSize, std::to_string(id).append(":l1"));
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, ptSize, std::to_string(id).append(":l2"));
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, ptSize, std::to_string(id).append(":l3"));
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

