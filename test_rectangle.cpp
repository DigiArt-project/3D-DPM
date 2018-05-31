// Written by Fisichella Thomas
// Date 25/05/2018

#include "Rectangle.h"
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

void displayCube(int width, int heigth, int depth){

    std::cout<<"Displaying..."<<std::endl;
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Scene"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem();
    //viewer->addCube(10, 20, 10, 20, 10, 20,1,1,1,"cube");
    Eigen::Vector3f translation(0,0,0);
    Eigen::Quaternionf rotation(0,0,0,0);
    viewer->addCube(translation, rotation, width, heigth, depth,"cube");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cube");
    while (!viewer->wasStopped())
        //while (!model_viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
}

void displayCube(int xmin, int xmax, int ymin, int ymax,int zmin, int zmax){
    
    std::cout<<"Displaying..."<<std::endl;
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Scene"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem();
    viewer->addCube(xmin, xmax, ymin, ymax, zmin, zmax,1,1,1,"cube");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cube");
    while (!viewer->wasStopped())
        //while (!model_viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
}

pcl::PointXYZ eigenVectorToPointPcl(Eigen::Vector3f pt){
    pcl::PointXYZ point;
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z();
    
    return point;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloudFromKeypointsRectangle(Rectangle& rectangle){
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ topLeftPoint, topRightPoint,bottomLeftPoint,bottomRightPoint, topBackLeftPoint, topBackRightPoint, bottomBackLeftPoint, bottomBackRightPoint;

    rectangle.changeToPclCoordinateSystem();
    
    topLeftPoint = eigenVectorToPointPcl(rectangle.topFrontLeft());
    topRightPoint = eigenVectorToPointPcl(rectangle.topFrontRight());
    bottomLeftPoint = eigenVectorToPointPcl(rectangle.bottomFrontLeft());
    bottomRightPoint = eigenVectorToPointPcl(rectangle.bottomFrontRight());
    topBackLeftPoint = eigenVectorToPointPcl(rectangle.topBackLeft());
    topBackRightPoint = eigenVectorToPointPcl(rectangle.topBackRight());
    bottomBackLeftPoint = eigenVectorToPointPcl(rectangle.bottomBackLeft());
    bottomBackRightPoint = eigenVectorToPointPcl(rectangle.bottomBackRight());

    basic_cloud_ptr->points.push_back (topLeftPoint);
    basic_cloud_ptr->points.push_back (topRightPoint);
    basic_cloud_ptr->points.push_back (bottomLeftPoint);
    basic_cloud_ptr->points.push_back (bottomRightPoint);
    basic_cloud_ptr->points.push_back (topBackLeftPoint);
    basic_cloud_ptr->points.push_back (topBackRightPoint);
    basic_cloud_ptr->points.push_back (bottomBackLeftPoint);
    basic_cloud_ptr->points.push_back (bottomBackRightPoint);
    return basic_cloud_ptr;
}

void displayCubeLine(Rectangle& rectangle){
    std::cout<<"Displaying..."<<std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Scene"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem(1,0,0,0);
    
    rectangle.changeToPclCoordinateSystem();
    
    viewer->addLine(eigenVectorToPointPcl(rectangle.topFrontLeft()), eigenVectorToPointPcl(rectangle.topFrontRight()),"l1");
    viewer->addLine(eigenVectorToPointPcl(rectangle.topFrontLeft()), eigenVectorToPointPcl(rectangle.topBackLeft()),"l2");
    viewer->addLine(eigenVectorToPointPcl(rectangle.topFrontLeft()), eigenVectorToPointPcl(rectangle.bottomFrontLeft()),"l3");
    
    viewer->addLine(eigenVectorToPointPcl(rectangle.bottomFrontLeft()), eigenVectorToPointPcl(rectangle.bottomFrontRight()),"l4");
    viewer->addLine(eigenVectorToPointPcl(rectangle.bottomFrontLeft()), eigenVectorToPointPcl(rectangle.bottomBackLeft()),"l5");
    
    viewer->addLine(eigenVectorToPointPcl(rectangle.topBackRight()), eigenVectorToPointPcl(rectangle.bottomBackRight()),"l6");
    viewer->addLine(eigenVectorToPointPcl(rectangle.topBackRight()), eigenVectorToPointPcl(rectangle.topFrontRight()),"l7");
    
    viewer->addLine(eigenVectorToPointPcl(rectangle.bottomBackLeft()), eigenVectorToPointPcl(rectangle.bottomBackRight()),"l8");
    viewer->addLine(eigenVectorToPointPcl(rectangle.bottomBackLeft()), eigenVectorToPointPcl(rectangle.topBackLeft()),"l9");
    
    viewer->addLine(eigenVectorToPointPcl(rectangle.topBackRight()), eigenVectorToPointPcl(rectangle.topBackLeft()),"l10");

    viewer->addLine(eigenVectorToPointPcl(rectangle.bottomFrontRight()), eigenVectorToPointPcl(rectangle.bottomBackRight()),"l11");
viewer->addLine(eigenVectorToPointPcl(rectangle.bottomFrontRight()), eigenVectorToPointPcl(rectangle.topFrontRight()),"l12");
    
    while (!viewer->wasStopped())
        //while (!model_viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void displayCube(Rectangle& rectangle){

    int xmin = rectangle.x();
    int xmax = xmin + rectangle.width();
    int ymin = rectangle.y();
    int ymax = ymin + rectangle.height();
    int zmin = rectangle.z();
    int zmax = zmin + rectangle.depth();
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>());
    basic_cloud_ptr =  createPointCloudFromKeypointsRectangle(rectangle);
    
    std::cout<<"Displaying..."<<std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Scene"));
    
    
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem(1,0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(basic_cloud_ptr,"keypoints");
    
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "keypoints");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");
    while (!viewer->wasStopped())
        //while (!model_viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}



//http://docs.pointclouds.org/1.7.2/search.php?query=addcube
int main(){
    int width = 5;
    int height = 5;
    int depth = 10;
    //3D XYZ coordinate of the first point
    int xmin = 0;
    int ymin = 0;
    int zmin = 0;
    
    Rectangle rect(xmin,ymin,zmin,width,height,depth);
    rect.toString();
    
    displayCubeLine(rect);
    //displayCube(rect);
}
