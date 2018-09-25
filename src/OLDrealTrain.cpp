#include "Mixture.h"
#include "Intersector.h"
#include "Object.h"


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
#include<pcl/io/ply_io.h>

#include <sys/timeb.h>
#include <dirent.h>

using namespace FFLD;
using namespace std;
using namespace Eigen;

int getMilliCount(){
    timeb tb;
    ftime(&tb);
    int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
    return nCount;
}

struct Detection : public Rectangle
{
    GSHOTPyramid::Scalar score;
    int x;
    int y;
    int z;
    int lvl;

    Detection() : score(0), x(0), y(0), z(0), lvl(0)
    {
    }

    Detection(GSHOTPyramid::Scalar score, int z, int y, int x, int lvl, Rectangle bndbox) : Rectangle(bndbox),
    score(score), x(x), y(y), z(z), lvl(lvl)
    {
    }

    bool operator<(const Detection & detection) const
    {
        return score > detection.score;
    }
};

class Test{
public:

    Test()
    {

        sceneResolution = 0.09;//25 * GSHOTPyramid::computeCloudResolution(sceneCloud);//0.0845292;//
        cout<<"test::sceneResolution : "<<sceneResolution<<endl;

    }

    void trainPositives(string folder){

        int nbParts = 4;
        triple<int, int, int> chairSize(6,4,5);//in lvl 1
        triple<int, int, int> chairPartSize( 6,4,5);//in lvl 0
        Model model( chairSize, 0);
        std::vector<Model> models = { model};

        vector<string> listFiles;
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (folder.c_str())) != NULL) {
          while ((ent = readdir (dir)) != NULL) {
              if( string(ent->d_name).compare(".") != 0 && string(ent->d_name).compare("..") != 0)
                listFiles.push_back( folder + ent->d_name);
//            printf ("%s\n", (folder + ent->d_name).c_str());
          }
          closedir (dir);
        } else {
          perror ("could not open directory");
        }

        vector<Scene> scenes(listFiles.size());
        for( int i=0; i < listFiles.size(); ++i){
            printf ("%s\n", listFiles[i].c_str());
            PointCloudPtr cloud( new PointCloudT);

            if (pcl::io::loadPLYFile<PointType>(listFiles[i].c_str(), *cloud) == -1) {
                cout<<"couldnt open ply file"<<endl;
            }

            PointType min;
            PointType max;
            pcl::getMinMax3D(*cloud, min, max);

            Vector3f resolution( (max.z - min.z)/(chairSize.first),
                                 (max.y - min.y)/(chairSize.second),
                                 (max.x - min.x)/(chairSize.third));

            Vector3i originScene = Vector3i(floor(min.z/resolution(0)),
                                   floor(min.y/resolution(1)),
                                   floor(min.x/resolution(2)));
            triple<int, int, int> sceneSize( ceil((max.z-originScene(0)*resolution(0))/resolution(0))+1,
                                                    ceil((max.y-originScene(1)*resolution(1))/resolution(1))+1,
                                                    ceil((max.x-originScene(2)*resolution(2))/resolution(2))+1);
            Rectangle chairBox(originScene , sceneSize.first, sceneSize.second, sceneSize.third, sceneResolution);

            Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, chairBox);
            scenes[i] = Scene( originScene, chairBox.depth(), chairBox.height(), chairBox.width(),
                               listFiles[i], {obj});
        }


        Mixture mixture( models);

        int interval = 1, nbIterations = 5, nbDatamine = 2, maxNegSample = 20;
        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, nbIterations/nbIterations,
                      nbDatamine, maxNegSample);


        PointCloudPtr chairCloud1( new PointCloudT);

        if (pcl::io::loadPLYFile<PointType>(scenes[0].filename().c_str(), *chairCloud1) == -1) {
            cout<<"couldnt open ply file"<<endl;
        }

        GSHOTPyramid pyramid(chairCloud1, Eigen::Vector3i( 3,3,3), interval);
        GSHOTPyramid::Level root2x;
        root2x = pyramid.levels()[0].block(0, 0, 0,
                                          scenes[0].depth(), scenes[0].height(), scenes[0].width());

        //TODO include initializeParts in train()
        mixture.initializeParts( nbParts, chairPartSize, root2x);

        for(int i=0; i < nbParts; ++i){
            cout<<"test::initializeParts offset["<<i+1<<"] = "<< mixture.models()[0].parts()[i+1].offset <<endl;
        }

        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval,
                      nbIterations, nbDatamine, maxNegSample);

    }


    float sceneResolution;
    Viewer viewer;
};


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


    Test test;

    // testSceneMiddle_compress
    // smallScene2
    int start = getMilliCount();


    test.trainPositives("/media/ubuntu/DATA/3DDataset/StructureSensor_normalized/chair/full/");


    int end = getMilliCount();

    cout << "Time : " << end-start << endl;


    test.viewer.show();




    return 0;
}

