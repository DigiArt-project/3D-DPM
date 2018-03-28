
#include "Mixture.h"

#include <cstdlib>
#include <sys/timeb.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


    char* filename = "/home/ubuntu/3DDataset/3DDPM/chair.pcd";
    PointCloudPtr cloud( new PointCloudT);
    if (pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
        return 0;
    }

    float starting_resolution = GSHOTPyramid::computeCloudResolution(cloud);



    PointType min;
    PointType max;
    pcl::getMinMax3D(*cloud , min, max);
    float resolution = 7 * starting_resolution;
    Model::triple<int, int, int> sceneSize( (max.z-min.z)/resolution+1,
                                           (max.y-min.y)/resolution+1,
                                           (max.x-min.x)/resolution+1);
    Model::triple<int, int, int> rootSize( sceneSize.first/4,
                                           sceneSize.second/4,
                                           sceneSize.third/4);
    Model::triple<int, int, int> partSize( sceneSize.first/6,
                                           sceneSize.second/6,
                                           sceneSize.third/6);
    Rectangle sceneRec( Eigen::Vector3i(1, 1, 1), rootSize.third, rootSize.second, rootSize.first);

    cout<<"test::sceneRec : "<< sceneRec <<endl;
    cout<<"test::sceneSize : "<<sceneSize.first<<" "<<sceneSize.second<<" "<<sceneSize.third<<endl;
    cout<<"test::rootSize : "<<rootSize.first<<" "<<rootSize.second<<" "<<rootSize.third<<endl;
    cout<<"test::partSize : "<<partSize.first<<" "<<partSize.second<<" "<<partSize.third<<endl;

    Model model( rootSize, 1, partSize);
    GSHOTPyramid pyramid(cloud, Eigen::Vector3i( 3,3,3));
    model.parts()[0].filter = pyramid.levels()[0].block( 1, 1, 1, rootSize.third, rootSize.second, rootSize.first);
    model.parts()[1].filter = pyramid.levels()[0].block( 2, 2, 1, partSize.third, partSize.second, partSize.first);
    std::vector<Model> models = { model};

    vector<Object> objects;
    Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec);
    objects.push_back(obj);
    vector<Scene> scenes = {Scene( sceneSize.third, sceneSize.second, rootSize.first, filename, objects)};


    Mixture mixture( models);
    mixture.zero_ = false;

    int interval = 1;
    float overlap = 0.4;
    vector<pair<Model, int> > positives;
//    mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval);
    mixture.posLatentSearch(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, overlap, positives);

    cout<<"test::positives[0].first.parts()[0].filter.isZero() : "<< GSHOTPyramid::TensorMap( positives[0].first.parts()[0].filter).isZero() << endl;
    //offset Null for the root
    cout<<"test::positives[0].first.parts()[0].offset : "<< positives[0].first.parts()[1].offset << endl;
    cout<<"test::positives[0].first.parts()[0].deformation : "<< positives[0].first.parts()[1].deformation << endl;



    ofstream out("tmp.txt");

    out << (positives[0].first);
    return 0;
}

