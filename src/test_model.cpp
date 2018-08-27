// Written by Fisichella Thomas
// Date 25/05/2018

#include "Model.h"
#include "Rectangle.h"
#include "viewer.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>

#include <cstdlib>
#include <sys/timeb.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;

    class Test{
    public:
        Test(){
        }

        void test1(){
            Eigen::Vector3i pad(1,1,1);
            Model::triple<int,int,int> rootSize(5,5,5);
            Model::triple<int,int,int>partSize( 10,10,10);
            Model model( rootSize);


            Model sample;
            PointCloudPtr cloud(new PointCloudT);
        //    pcl::io::loadPCDFile( "/home/ubuntu/3DDataset/3DDPM/scene.pcd", *cloud);
            pcl::io::loadPCDFile( "/home/ubuntu/3DDataset/3DDPM/chair.pcd", *cloud);

            int lvl = 0, x = 0, y = 0, z= 0;
            GSHOTPyramid scenePyramid(cloud, pad);
            model.initializeSample(scenePyramid, x, y, z, lvl,sample);

            cout<< scenePyramid.levels()[1].size() <<endl;

            GSHOTPyramid::Level root2x = scenePyramid.levels()[0];


             model.initializeParts( 2, partSize, root2x);

        //    cout<< cloud->size() <<endl;
            vector<Tensor3DF> scores(5);

            //TODO
            sample.convolve(scenePyramid, scores);

            cout<< "score : "<<scores[0].size()<<endl;
        }

        void testDT3D(){

        }

        void initSample(){
            Eigen::Vector3i pad(1,1,1);


            PointCloudPtr cloud(new PointCloudT);
            pcl::io::loadPCDFile( "/home/ubuntu/3DDataset/3DDPM/smallScene.pcd", *cloud);

            float sceneResolution = 25 * GSHOTPyramid::computeCloudResolution(cloud);
            cout<<"test::sceneResolution : "<<sceneResolution<<endl;


            Eigen::Vector3i origin(-2, -1, 4);//-2,10,4
            Model::triple<int, int, int> chairSize(5,3,4);
//            Model::triple<int, int, int> chairPartSize( chairBox.depth()/2,
//                                                        chairBox.height()/2,
//                                                        chairBox.width()/2);

//            cout<<"test::chairPartSize : "<<chairPartSize.first<<" "<<chairPartSize.second<<" "<<chairPartSize.third<<endl;

            Model model( chairSize, 0);


            Model sample;


            Rectangle boxFound(origin, chairSize.first, chairSize.second, chairSize.third, sceneResolution*2);

            GSHOTPyramid scenePyramid(cloud, pad, 1);
            model.initializeSample(scenePyramid, origin(0)+2, origin(1)+3, origin(2)+3, 1, sample);

            ofstream out("initSampleScene1.txt");

            out << (sample);


            PointCloudPtr subspace(new PointCloudT());
            pcl::UniformSampling<PointType> sampling;
            sampling.setInputCloud(cloud);
            sampling.setRadiusSearch (sceneResolution);
            sampling.filter(*subspace);

            Viewer viewer;
            viewer.addPC( subspace);
            viewer.displayCubeLine(boxFound);

            viewer.show();
        }

        void compareInitSampleResults(){

            ifstream in1("initSampleScene1.txt");

            if (!in1.is_open()) {
                cerr << "Cannot open model file\n" << endl;
                return;
            }

             Model model1;
             in1 >> model1;

             ifstream in2("initSampleTable1.txt");

             if (!in2.is_open()) {
                 cerr << "Cannot open model file\n" << endl;
                 return;
             }

              Model model2;
              in2 >> model2;


              cout<<"score compare : "<<model1.dot(model2)<<endl;




        }


    private:
        std::vector<Model::Part> parts_;
        double bias_;
    };

int main(){

    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


    Test test;
    test.initSample();
//    test.compareInitSampleResults();




    
    return 0;
}

