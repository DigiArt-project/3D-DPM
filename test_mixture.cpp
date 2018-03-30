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

    Test( char* sceneFilename, char* modelFilename) :
        sceneName( sceneFilename), sceneCloud( new PointCloudT), modelCloud( new PointCloudT)
    {

        if (pcl::io::loadPCDFile<PointType>(sceneFilename, *sceneCloud) == -1) {
            cout<<"test::couldnt open pcd file"<<endl;
        }
        if (pcl::io::loadPCDFile<PointType>(modelFilename, *modelCloud) == -1) {
            cout<<"test::couldnt open pcd file"<<endl;
        }
        starting_resolution = GSHOTPyramid::computeCloudResolution(sceneCloud);


        PointType min;
        PointType max;
        pcl::getMinMax3D(*sceneCloud , min, max);
        resolution = 7 * starting_resolution;
        sceneSize = Model::triple<int, int, int>( (max.z-min.z)/resolution+1, (max.y-min.y)/resolution+1, (max.x-min.x)/resolution+1);
        cout<<"test::sceneSize : "<<sceneSize.first<<" "<<sceneSize.second<<" "<<sceneSize.third<<endl;
    }

    void testNegLatSearch(){
        cout << "testNegLatSearch ..." << endl;
        Model::triple<int, int, int> rootSize( sceneSize.first/4,
                                               sceneSize.second/4,
                                               sceneSize.third/4);
        Model::triple<int, int, int> partSize( sceneSize.first/6,
                                               sceneSize.second/6,
                                               sceneSize.third/6);
        Rectangle sceneRec( Eigen::Vector3i(1, 1, 1), rootSize.third, rootSize.second, rootSize.first);
        Rectangle sceneRec2( Eigen::Vector3i(2, 2, 2), rootSize.third*0.75, rootSize.second*2, rootSize.first);

        cout<<"test::sceneRec : "<< sceneRec <<endl;
        cout<<"test::sceneRec2 : "<< sceneRec2 <<endl;
        cout<<"test::sceneSize : "<<sceneSize.first<<" "<<sceneSize.second<<" "<<sceneSize.third<<endl;
        cout<<"test::rootSize : "<<rootSize.first<<" "<<rootSize.second<<" "<<rootSize.third<<endl;
        cout<<"test::partSize : "<<partSize.first<<" "<<partSize.second<<" "<<partSize.third<<endl;

        Model model( rootSize, 1, partSize);
        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3));
        model.parts()[0].filter = pyramid.levels()[0].block( 1, 1, 1, rootSize.first, rootSize.second, rootSize.third);
        model.parts()[1].filter = pyramid.levels()[0].block( 2, 2, 1, partSize.first, partSize.second, partSize.third);
        std::vector<Model> models = { model};

        vector<Object> objects;
        Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec);
        Object obj2(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec2);
        objects.push_back(obj);
        objects.push_back(obj2);

        vector<Scene> scenes = {Scene( sceneSize.third, sceneSize.second, rootSize.first, sceneName, objects)};


        Mixture mixture( models);
        mixture.zero_ = false;

        int interval = 1;
        int maxNegatives = 24000;
        vector<pair<Model, int> >  negatives;

        int j = 0;

        for (int i = 0; i < negatives.size(); ++i)
            if ((negatives[i].first.parts()[0].deformation(3) =
                 models[negatives[i].second].dot(negatives[i].first)) > -1.01)
                negatives[j++] = negatives[i];

        negatives.resize(j);
        mixture.negLatentSearch(scenes, Object::BICYCLE, Eigen::Vector3i( 3,3,3), interval, maxNegatives, negatives);
        cout<<"test::negatives.size = "<<negatives.size()<<endl;
        cout<<"test::negatives[0].first.parts()[0].filter.isZero() : "<< GSHOTPyramid::TensorMap( negatives[0].first.parts()[0].filter).isZero() << endl;
    //    //offset Null for the root
    //    cout<<"test::positives[0].first.parts()[0].offset : "<< positives[0].first.parts()[1].offset << endl;
    //    cout<<"test::positives[0].first.parts()[0].deformation : "<< positives[0].first.parts()[1].deformation << endl;



        ofstream out("tmp.txt");

        out << (negatives[0].first);
    }

    void testPosLatSearch(){

        Model::triple<int, int, int> rootSize( sceneSize.first/4,
                                               sceneSize.second/4,
                                               sceneSize.third/4);
        Model::triple<int, int, int> partSize( sceneSize.first/6,
                                               sceneSize.second/6,
                                               sceneSize.third/6);
        Rectangle sceneRec( Eigen::Vector3i(1, 1, 1), rootSize.third, rootSize.second, rootSize.first);
        Rectangle sceneRec2( Eigen::Vector3i(7,7,7), rootSize.third/**0.75*/, rootSize.second/**2*/, rootSize.first);

        cout<<"test::sceneRec : "<< sceneRec <<endl;
        cout<<"test::sceneRec2 : "<< sceneRec2 <<endl;
        cout<<"test::rootSize : "<<rootSize.first<<" "<<rootSize.second<<" "<<rootSize.third<<endl;
        cout<<"test::partSize : "<<partSize.first<<" "<<partSize.second<<" "<<partSize.third<<endl;

        Model model( rootSize, 1, partSize);
        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3));
        model.parts()[0].filter = pyramid.levels()[0].block( 6, 1, 1, rootSize.first, rootSize.second, rootSize.third);
        model.parts()[1].filter = pyramid.levels()[0].block( 7, 2, 1, partSize.first, partSize.second, partSize.third);
        std::vector<Model> models = { model};

        vector<Object> objects;
        Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec);
        Object obj2(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec2);
        objects.push_back(obj);
        objects.push_back(obj2);

        vector<Scene> scenes = {Scene( sceneSize.third, sceneSize.second, rootSize.first, sceneName, objects)};


        Mixture mixture( models);
        mixture.zero_ = false;

        int interval = 1;
        float overlap = 0.4;
        vector<pair<Model, int> > positives;
    //    mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval);
        mixture.posLatentSearch(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, overlap, positives);

        cout<<"test::positives[0].first.parts()[0].filter.isZero() : "<< GSHOTPyramid::TensorMap( positives[0].first.parts()[0].filter).isZero() << endl;
    //    //offset Null for the root
        cout<<"test::positives[0].first.parts()[0].offset : "<< positives[0].first.parts()[1].offset << endl;
    //    cout<<"test::positives[0].first.parts()[0].deformation : "<< positives[0].first.parts()[1].deformation << endl;



        ofstream out("tmp.txt");

        out << (positives[0].first);
    }

    Mixture testTrain( Model::triple<int, int, int> rootSize, Rectangle sceneRec){


        Model::triple<int, int, int> partSize( rootSize.first/3,
                                               rootSize.second/3,
                                               rootSize.third/3);
//        Rectangle sceneRec( Eigen::Vector3i(1, 1, 1), rootSize.third, rootSize.second, rootSize.first);
//        Rectangle sceneRec2( Eigen::Vector3i(2, 2, 2), rootSize.third*0.75, rootSize.second*2, rootSize.first);

        cout<<"test::sceneRec : "<< sceneRec <<endl;
//        cout<<"test::sceneRec2 : "<< sceneRec2 <<endl;
        cout<<"test::rootSize : "<<rootSize.first<<" "<<rootSize.second<<" "<<rootSize.third<<endl;
        cout<<"test::partSize : "<<partSize.first<<" "<<partSize.second<<" "<<partSize.third<<endl;

        Model model( rootSize, 1, partSize);
        std::vector<Model> models = { model};

        vector<Object> objects;
        Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec);
//        Object obj2(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec2);
        objects.push_back(obj);
//        objects.push_back(obj2);

        vector<Scene> scenes = {Scene( sceneSize.third, sceneSize.second, rootSize.first, sceneName, objects)};


        Mixture mixture( models);

        int interval = 1;
        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval);

        return mixture;

    }

    void testTrainSVM(){

        Model::triple<int, int, int> rootSize( sceneSize.first/4,
                                           sceneSize.second/4,
                                               sceneSize.third/4);
        Model::triple<int, int, int> partSize( sceneSize.first/6,
                                               sceneSize.second/6,
                                               sceneSize.third/6);
        Rectangle sceneRec( Eigen::Vector3i(1, 1, 1), rootSize.third, rootSize.second, rootSize.first);
        Rectangle sceneRec2( Eigen::Vector3i(3, 5, 4), rootSize.third, rootSize.second, rootSize.first);
//        Rectangle sceneRec3( Eigen::Vector3i(2, 2, 2), rootSize.third*0.75, rootSize.second*2, rootSize.first);

        cout<<"test::sceneRec : "<< sceneRec <<endl;
        cout<<"test::sceneRec2 : "<< sceneRec2 <<endl;
        cout<<"test::rootSize : "<<rootSize.first<<" "<<rootSize.second<<" "<<rootSize.third<<endl;
        cout<<"test::partSize : "<<partSize.first<<" "<<partSize.second<<" "<<partSize.third<<endl;

        vector<pair<Model, int> >  positives, negatives;
        Model modelScene( rootSize, 1, partSize);
        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3));

        modelScene.parts()[0].filter = pyramid.levels()[0].block( 1, 1, 1, rootSize.first, rootSize.second, rootSize.third);
        modelScene.parts()[1].filter = pyramid.levels()[0].block( 2, 2, 1, partSize.first, partSize.second, partSize.third);

        positives.push_back(pair<Model, int>(modelScene, 0));

        modelScene.parts()[0].filter = pyramid.levels()[0].block( 5, 5, 5, rootSize.first, rootSize.second, rootSize.third);
        modelScene.parts()[1].filter = pyramid.levels()[0].block( 5, 5, 5, partSize.first, partSize.second, partSize.third);

        negatives.push_back(pair<Model, int>(modelScene, 0));


        Model model( rootSize, 1, partSize);
        std::vector<Model> models = { model};

        Mixture mixture( models);

        double C = 0.002;
        double J = 2.0;
        mixture.trainSVM(positives, negatives, C, J);

        ofstream out("tmp.txt");

        out << (mixture);

        ////////////

        modelScene.parts()[0].filter = pyramid.levels()[0].block( 5, 1, 1, rootSize.first, rootSize.second, rootSize.third);
        modelScene.parts()[1].filter = pyramid.levels()[0].block( 5, 2, 1, partSize.first, partSize.second, partSize.third);

        positives[0] = pair<Model, int>(modelScene, 0);

        modelScene.parts()[0].filter = pyramid.levels()[0].block( 0, 0, 5, rootSize.first, rootSize.second, rootSize.third);
        modelScene.parts()[1].filter = pyramid.levels()[0].block( 1, 1, 5, partSize.first, partSize.second, partSize.third);

        negatives[0] = pair<Model, int>(modelScene, 0);

        mixture.trainSVM(positives, negatives, C, J);

        ofstream out2("tmp2.txt");

        out2 << (mixture);
    }

    char* sceneName;
    PointCloudPtr sceneCloud;
    PointCloudPtr modelCloud;
    float starting_resolution;
    float resolution;
    Model::triple<int, int, int> sceneSize;
};


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

/////////Construct scene
    PointCloudPtr sceneCloud( new PointCloudT);
    PointCloudPtr chairCloud( new PointCloudT);
    PointCloudPtr tableCloud( new PointCloudT);
    PointCloudPtr tmpCloud( new PointCloudT);

    if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/3DDataset/3DDPM/chair.pcd", *chairCloud) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/3DDataset/3DDPM/table.pcd", *tableCloud) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }
    if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/3DDataset/3DDPM/smallScene.pcd", *sceneCloud) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
    }

    float sceneResolution = 30 * GSHOTPyramid::computeCloudResolution(sceneCloud);

    float chairResolution = sceneResolution;//30 * GSHOTPyramid::computeCloudResolution(chairCloud);
    float tableTesolution = sceneResolution;//30 * GSHOTPyramid::computeCloudResolution(tableCloud);

    cout<<"test::sceneResolution : "<<sceneResolution<<endl;
    cout<<"test::chairResolution : "<<chairResolution<<endl;
    cout<<"test::tableTesolution : "<<tableTesolution<<endl;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform.translation() << 1, 0.0, 0.0;

    pcl::transformPointCloud (*chairCloud, *tmpCloud, transform);

    pcl::UniformSampling<PointType> sampling;
    sampling.setInputCloud(tmpCloud);
    sampling.setRadiusSearch (chairResolution);
    sampling.filter(*sceneCloud);

    PointType minChair;
    PointType maxChair;
    pcl::getMinMax3D(*sceneCloud , minChair, maxChair);


    Model::triple<int, int, int> chairSize( (maxChair.z-minChair.z)/chairResolution+1,
                                            (maxChair.y-minChair.y)/chairResolution+1,
                                            (maxChair.x-minChair.x)/chairResolution+1);

    Rectangle chairBox(Eigen::Vector3i(0,0,0), chairSize.third, chairSize.second, chairSize.first);

    tmpCloud = tableCloud;
    sampling.setInputCloud(tmpCloud);
    sampling.setRadiusSearch (tableTesolution);
    sampling.filter(*tableCloud);

    PointType minTable;
    PointType maxTable;
    pcl::getMinMax3D(*tableCloud , minTable, maxTable);

    Model::triple<int, int, int> tableSize( (maxTable.z-minTable.z)/tableTesolution+1,
                                            (maxTable.y-minTable.y)/tableTesolution+1,
                                            (maxTable.x-minTable.x)/tableTesolution+1);

    Rectangle tableBox(Eigen::Vector3i(0,0,0), tableSize.third, tableSize.second, tableSize.first);


    *sceneCloud += *tableCloud;
///////////////////////////


    Test test( "/home/ubuntu/3DDataset/3DDPM/smallScene.pcd", "/home/ubuntu/3DDataset/3DDPM/chair.pcd");

    //    test.testNegLatSearch();
    //test.testTrainSVM();
    Mixture mix = test.testTrain(chairSize, chairBox);
//    test.testPosLatSearch();

    Viewer viewer;
    viewer.addPC( sceneCloud);
    viewer.displayCubeLine(chairBox, chairResolution, minChair);
    Rectangle expChairBox( Eigen::Vector3i(mix.models_[0].parts()[0].offset(0),
                                               mix.models_[0].parts()[0].offset(1),
                                               mix.models_[0].parts()[0].offset(2)),
                            chairSize.third, chairSize.second, chairSize.first);
    viewer.displayCubeLine(expChairBox, chairResolution, minChair, Eigen::Vector3i(255,255,0));

    viewer.show();



    return 0;
}

