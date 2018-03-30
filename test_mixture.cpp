#include "Mixture.h"

#include <cstdlib>
#include <sys/timeb.h>


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

    void testTrain(){

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
        cout<<"test::rootSize : "<<rootSize.first<<" "<<rootSize.second<<" "<<rootSize.third<<endl;
        cout<<"test::partSize : "<<partSize.first<<" "<<partSize.second<<" "<<partSize.third<<endl;

        Model model( rootSize, 1, partSize);
        std::vector<Model> models = { model};

        vector<Object> objects;
        Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec);
        Object obj2(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec2);
        objects.push_back(obj);
        objects.push_back(obj2);

        vector<Scene> scenes = {Scene( sceneSize.third, sceneSize.second, rootSize.first, sceneName, objects)};


        Mixture mixture( models);

        int interval = 1;
        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval);

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

    cout << "Start ..." << endl;
    Test test( "/home/ubuntu/3DDataset/3DDPM/chair.pcd", "/home/ubuntu/3DDataset/3DDPM/chair.pcd");

    //    test.testNegLatSearch();
    //test.testTrainSVM();
//    test.testTrain();
    test.testPosLatSearch();

    return 0;
}

