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

#include <sys/timeb.h>


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

    Test( char* sceneFilename, char* chairFilename, char* tableFilename) :
        sceneName( sceneFilename), tableName( tableFilename),
        sceneCloud( new PointCloudT), chairCloud( new PointCloudT), tableCloud( new PointCloudT)
    {


        if (readPointCloud(sceneFilename, *sceneCloud) == -1) {
            cout<<"test::couldnt open pcd file"<<endl;
        }
        PointCloudPtr tmpCloud( new PointCloudT);
        if (readPointCloud(chairFilename, *tmpCloud) == -1) {
            cout<<"test::couldnt open pcd file"<<endl;
        }
        if (readPointCloud(tableFilename, *tableCloud) == -1) {
            cout<<"test::couldnt open pcd file"<<endl;
        }


        sceneResolution = 0.09;//25 * GSHOTPyramid::computeCloudResolution(sceneCloud);//0.0845292;//
        cout<<"test::sceneResolution : "<<sceneResolution<<endl;

        viewer.viewer->addLine(pcl::PointXYZ(0,-1,0), pcl::PointXYZ(2*sceneResolution, -1, 0),"ijbij");

        PointType minScene;
        PointType maxScene;
        pcl::getMinMax3D(*sceneCloud , minScene, maxScene);

        originScene = Vector3i(floor(minScene.z/sceneResolution),
                               floor(minScene.y/sceneResolution),
                               floor(minScene.x/sceneResolution));
        Model::triple<int, int, int> sceneSize( ceil((maxScene.z-originScene(0)*sceneResolution)/sceneResolution)+1,
                                                ceil((maxScene.y-originScene(1)*sceneResolution)/sceneResolution)+1,
                                                ceil((maxScene.x-originScene(2)*sceneResolution)/sceneResolution)+1);
        sceneBox = Rectangle(originScene , sceneSize.first, sceneSize.second, sceneSize.third, sceneResolution);
        cout<<"test:: sceneBox = "<<sceneBox<<endl;
        cout<<"test:: minScene = "<<minScene<<endl;
        cout<<"test:: maxScene = "<<maxScene<<endl;


        PointCloudPtr subspace(new PointCloudT());
        pcl::UniformSampling<PointType> sampling;
        sampling.setInputCloud(sceneCloud);
        sampling.setRadiusSearch (sceneResolution);
        sampling.filter(*subspace);

        viewer.addPC( subspace, 3);
        viewer.displayCubeLine(sceneBox);
//        viewer.viewer->addLine(minScene,
//                pcl::PointXYZ(sceneResolution*(originScene(2)+sceneSize.third),
//                              sceneResolution*(originScene(1)+sceneSize.second),
//                              sceneResolution*(originScene(0)+sceneSize.first)), "kjbij");




        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        // Define a translation of 2.5 meters on the x axis.
        if( sceneName == "/home/ubuntu/3DDataset/3DDPM/smallScene1.pcd"){
            transform.translation() << 2, 0.0, 0.0;
        } else if( sceneName == "/home/ubuntu/3DDataset/3DDPM/smallScene2.pcd"){
            transform.translation() << 1, 2.0, 0.0;
        }
        pcl::transformPointCloud (*tmpCloud, *chairCloud, transform);


        PointType minChair;
        PointType maxChair;
        pcl::getMinMax3D(*chairCloud , minChair, maxChair);


        Model::triple<int, int, int> chairSize( ceil((maxChair.z-minChair.z)/sceneResolution/2)+1,
                                                ceil((maxChair.y-minChair.y)/sceneResolution/2)+1,
                                                ceil((maxChair.x-minChair.x)/sceneResolution/2)+1);
        chairBox = Rectangle(Eigen::Vector3i(round((minChair.z-minScene.z)/sceneResolution/2.0)-3,
                           round((minChair.y-minScene.y)/sceneResolution/2.0)-4,
                           round((minChair.x-minScene.x)/sceneResolution/2.0)-4),
                             chairSize.first, chairSize.second, chairSize.third, sceneResolution*2);


        cout<<"test:: chairBox = "<<chairBox<<endl;
        cout<<"test:: minChair = "<<minChair<<endl;
        cout<<"test:: maxChair = "<<maxChair<<endl;
        viewer.displayCubeLine(chairBox, Eigen::Vector3f(sceneResolution,sceneResolution,sceneResolution),
                               Eigen::Vector3i(255,255,0));


        PointType minTable;
        PointType maxTable;
        pcl::getMinMax3D(*tableCloud , minTable, maxTable);

        Model::triple<int, int, int> tableSize( ceil((maxTable.z-minTable.z)/sceneResolution/2)+1,
                                                ceil((maxTable.y-minTable.y)/sceneResolution/2)+1,
                                                ceil((maxTable.x-minTable.x)/sceneResolution/2)+1);



        tableBox = Rectangle(Eigen::Vector3i(floor(minTable.z/sceneResolution/2),
                                 floor(minTable.y/sceneResolution/2),
                                 floor(minTable.x/sceneResolution/2)),
                             tableSize.first, tableSize.second, tableSize.third, sceneResolution*2);

        cout<<"test:: tableBox = "<<tableBox<<endl;
        cout<<"test:: tableBox left = "<<tableBox.left()<<endl;
        cout<<"test:: tableBox right = "<<tableBox.right()<<endl;


    }

    void createChairModel(){

        int nbParts = 1;

        Model::triple<int, int, int> chairSize(chairBox.depth(), chairBox.height(), chairBox.width());
        Model::triple<int, int, int> chairPartSize( chairBox.depth()/2,
                                                    chairBox.height()/2,
                                                    chairBox.width()/2);

        Model model( chairSize, nbParts, chairPartSize);
        GSHOTPyramid pyramid(chairCloud, Eigen::Vector3i( 3,3,3));
        model.parts()[0].filter = pyramid.levels()[0].block( chairBox.origin()(0), chairBox.origin()(1), chairBox.origin()(2),
                                                             chairSize.first, chairSize.second, chairSize.third);
        model.parts()[1].filter = pyramid.levels()[0].block( 2, 0, 6, chairPartSize.first, chairPartSize.second, chairPartSize.third);

        ofstream out2("chairModel.txt");

        out2 << (model);
    }

    void testNegLatSearch(){
        cout << "testNegLatSearch ..." << endl;
        Model::triple<int, int, int> chairSize(chairBox.depth(), chairBox.height(), chairBox.width());
        Model::triple<int, int, int> chairPartSize( chairBox.depth()/2*2,
                                                    chairBox.height()/2*2,
                                                    chairBox.width()/2*2);

        cout<<"test::chairPartSize : "<<chairPartSize.first<<" "<<chairPartSize.second<<" "<<chairPartSize.third<<endl;

        Model model( chairSize, 1, chairPartSize);
//        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3));
        std::vector<Model> models = { model};

        vector<Object> objects;
        Object obj(Object::AEROPLANE, Object::Pose::UNSPECIFIED, false, false, chairBox);
//        Object obj2(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, chairBox);
        objects.push_back(obj);
//        objects.push_back(obj2);

        vector<Scene> scenes = {Scene( originScene, sceneBox.depth(), sceneBox.height(), sceneBox.width(),
                                tableName, objects)};

//        Mixture mixture( models);

//        GSHOTPyramid::Level root2x =
//                pyramid.levels()[0].block(chairBox.origin()(0)*2, chairBox.origin()(1)*2, chairBox.origin()(2)*2,
//                                          chairSize.first*2, chairSize.second*2, chairSize.third*2);
//        for(int mod = 0; mod<mixture.models().size(); ++mod){
//            mixture.models()[mod].initializeParts( mixture.models()[mod].parts().size() - 1,
//                                                           mixture.models()[mod].partSize(), root2x);
//        }

//        Model sample;
//        sample.parts().resize(1);
//        sample.parts()[0].filter = pyramid.levels()[0].block(chairBox.origin()(0), chairBox.origin()(1), chairBox.origin()(2),
//                                                              chairSize.first, chairSize.second, chairSize.third);
//        sample.parts()[0].offset.setZero();
//        sample.parts()[0].deformation.setZero();
//        mixture.models()[0].parts()[0].filter = sample.parts()[0].filter;



        ifstream in("tmp.txt");

        if (!in.is_open()) {
            cerr << "Cannot open model file\n" << endl;
            return;
        }

        Mixture mixture;
        in >> mixture;
//        mixture.train_ = false;

        if (mixture.empty()) {
            cerr << "Invalid model file\n" << endl;
            return;
        }

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
        mixture.negLatentSearch(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, maxNegatives, negatives);
        cout<<"test::negatives.size = "<<negatives.size()<<endl;
        if(negatives.size()>0){
            cout<<"test::negatives[0].first.parts()[0].filter.isZero() : "<< GSHOTPyramid::TensorMap( negatives[0].first.parts()[0].filter).isZero() << endl;
            //    //offset Null for the root
            //    cout<<"test::positives[0].first.parts()[0].offset : "<< positives[0].first.parts()[1].offset << endl;
            //    cout<<"test::positives[0].first.parts()[0].deformation : "<< positives[0].first.parts()[1].deformation << endl;



            ofstream out("tmp2.txt");

            out << (negatives[0].first);
        }
    }

    void testPosLatSearch(){

        int nbParts = 1;
        int nbLevel = 2;
        Model::triple<int, int, int> chairSize(chairBox.depth(), chairBox.height(), chairBox.width());
        Model::triple<int, int, int> chairPartSize( chairBox.depth()/2,
                                                    chairBox.height()/2,
                                                    chairBox.width()/2);

        cout<<"test::chairPartSize : "<<chairPartSize.first<<" "<<chairPartSize.second<<" "<<chairPartSize.third<<endl;

//        Rectangle chairBox2( chairBox);


        Model model( chairSize, nbParts, chairPartSize);
//        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3));
//        model.parts()[0].filter = pyramid.levels()[0].block( 6, 1, 1, chairSize.first, chairSize.second, chairSize.third);
//        model.parts()[1].filter = pyramid.levels()[0].block( 7, 2, 1, chairPartSize.first, chairPartSize.second, chairPartSize.third);
        std::vector<Model> models = { model};

        vector<Object> objects;
        Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, chairBox);
        Object obj2(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, tableBox);
        objects.push_back(obj);
        objects.push_back(obj2);

        vector<Scene> scenes = {Scene( originScene, sceneBox.depth(), sceneBox.height(), sceneBox.width(), sceneName, objects)};


        Mixture mixture( models);


        int interval = 1;
        float overlap = 0.4;
        vector<pair<Model, int> > positives;

        mixture.posLatentSearch(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, overlap, positives);

        cout<<"test::positives.size : "<< positives.size() << endl;
        mixture.zero_ = false;
        cout<<"test:: set zero_ to false" << endl;

        Model sample;
        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3), interval);
        vector<vector<Model::Positions> > positions;
        positions.resize(nbParts);
        for(int i=0; i<nbParts; ++i){
            (positions)[i].resize(nbLevel);
        }

//        mixture.models()[0].initializeSample(pyramid, chairBox.origin()(2), chairBox.origin()(1), chairBox.origin()(0), 0, sample, &positions);//x, y, z, lvl
        sample.parts().resize(1);
        sample.parts()[0].filter = pyramid.levels()[0].block(chairBox.origin()(0), chairBox.origin()(1), chairBox.origin()(2),
                                                              chairSize.first, chairSize.second, chairSize.third);
        sample.parts()[0].offset.setZero();
        sample.parts()[0].deformation.setZero();
        mixture.models()[0].parts()[0].filter = sample.parts()[0].filter;


        cout<<"test:: sample.parts()[0].filter.isZero() : "
           << GSHOTPyramid::TensorMap( sample.parts()[0].filter).isZero() << endl;
        cout<<"test:: mixture.models()[0].parts()[0].filter.isZero() : "
           << GSHOTPyramid::TensorMap( mixture.models()[0].parts()[0].filter).isZero() << endl;



        mixture.posLatentSearch(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, overlap, positives);


        if(positives.size()>0) {
            cout<<"test::positives[0].first.parts()[0].filter.isZero() : "<< GSHOTPyramid::TensorMap( positives[0].first.parts()[0].filter).isZero() << endl;
    //    //offset Null for the root
            cout<<"test::positives[0].first.parts()[0].offset : "<< positives[0].first.parts()[1].offset << endl;
    //    cout<<"test::positives[0].first.parts()[0].deformation : "<< positives[0].first.parts()[1].deformation << endl;
            ofstream out("tmp.txt");
            out << (positives[0].first);

            Eigen::Vector3i origin(-2, -1, 4);//check comment : Mix:PosLatentSearch found a positive sample at : -2 -1 4 / 0.169058

            Rectangle boxFound(origin, chairSize.first, chairSize.second, chairSize.third, sceneResolution*2);
            viewer.displayCubeLine(boxFound, Eigen::Vector3f(0,0,0), Eigen::Vector3i(255,0,255));

            //Mix:PosLatentSearch found a positive sample with offsets : -2 -3 -3
            Eigen::Vector3i partOrigin(origin(0) * 2 + positives[0].first.parts()[1].offset(0)/*-2*2*/,
                                       origin(1) * 2 + positives[0].first.parts()[1].offset(1)/*-3*2*/,
                                       origin(2) * 2 + positives[0].first.parts()[1].offset(2)/*-3*2*/);
            Rectangle partsBoxFound(partOrigin, chairPartSize.first, chairPartSize.second, chairPartSize.third, sceneResolution);

            viewer.displayCubeLine(partsBoxFound, Eigen::Vector3f(0,0,0), Eigen::Vector3i(0,255,0));

        }



    }

    void testTrain(){

        int nbParts = 5;


        Model::triple<int, int, int> chairSize(chairBox.depth(), chairBox.height(), chairBox.width());
        Model::triple<int, int, int> chairPartSize( chairBox.depth()/2*2,
                                                    chairBox.height()/2*2,
                                                    chairBox.width()/2*2);//in lvl 0

        cout<<"test::chairPartSize : "<<chairPartSize.first<<" "<<chairPartSize.second<<" "<<chairPartSize.third<<endl;

        Model model( chairSize, 0);
        std::vector<Model> models = { model};

        vector<Object> objects, objects2;
        Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, chairBox);
        Object obj2(Object::TRAIN, Object::Pose::UNSPECIFIED, false, false, tableBox);

        objects.push_back(obj);
        objects2.push_back(obj2);

        vector<Scene> scenes = {
            Scene( originScene, sceneBox.depth(), sceneBox.height(), sceneBox.width(), sceneName, objects)/*,*/
//            Scene( originScene, sceneBox.depth(), sceneBox.height(), sceneBox.width(), sceneName, objects),
//            Scene( originScene, sceneBox.depth(), sceneBox.height(), sceneBox.width(), sceneName, objects),
//            Scene( originScene, sceneBox.depth(), sceneBox.height(), sceneBox.width(), sceneName, objects),
//            Scene( originScene, sceneBox.depth(), sceneBox.height(), sceneBox.width(), tableName, objects2),
//            Scene( originScene, sceneBox.depth(), sceneBox.height(), sceneBox.width(), sceneName, objects)
        };


        Mixture mixture( models);

        int interval = 1, nbIterations = 5, nbDatamine = 2, maxNegSample = 20;
        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, nbIterations/nbIterations,
                      nbDatamine, maxNegSample);

        cout << "test:: root filter initialized" << endl;

        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3), interval);
        GSHOTPyramid::Level root2x;

        if( sceneName == "/home/ubuntu/3DDataset/3DDPM/smallScene1.pcd"){
            root2x = pyramid.levels()[0].block(chairBox.origin()(0)*2-sceneBox.origin()(0),
                                              chairBox.origin()(1)*2-sceneBox.origin()(1),
                                              chairBox.origin()(2)*2-sceneBox.origin()(2),
                                              chairSize.first*2, chairSize.second*2, chairSize.third*2);
        } else if( sceneName == "/home/ubuntu/3DDataset/3DDPM/smallScene2.pcd"){

//            Rectangle chairBox0(Eigen::Vector3i(round((minChair.z-minScene.z)/sceneResolution)-5,
//                               round((minChair.y-minScene.y)/sceneResolution)-7,
//                               round((minChair.x-minScene.x)/sceneResolution)-7),
//                                 chairSize.first*2-1, chairSize.second*2-1, chairSize.third*2-1, sceneResolution);
            root2x = pyramid.levels()[0].block(0,
                                              13*2,
                                              7*2,
                                              chairSize.first*2-1, chairSize.second*2-1, chairSize.third*2-1);
        }



        cout<<"test::initializeParts root2x.z = "<< chairBox.origin()(0)*2-sceneBox.origin()(0) <<endl;
        cout<<"test::initializeParts root2x.y = "<< chairBox.origin()(1)*2-sceneBox.origin()(1) <<endl;
        cout<<"test::initializeParts root2x.x = "<< chairBox.origin()(2)*2-sceneBox.origin()(2) <<endl;

        cout<<"test::initializeParts root2x.depths() = "<< root2x.depths() <<endl;
        cout<<"test::initializeParts root2x.rows() = "<< root2x.rows() <<endl;
        cout<<"test::initializeParts root2x.cols() = "<< root2x.cols() <<endl;

//        cout<<"test::initializeParts chairBox = "<< chairBox <<endl;
//        cout<<"test::initializeParts chairBox.origin()(0)*2 = "<< chairBox.origin()(0)*2 <<endl;
//        cout<<"test::initializeParts chairBox.origin()(1)*2 = "<< chairBox.origin()(1)*2 <<endl;
//        cout<<"test::initializeParts chairBox.origin()(2)*2 = "<< chairBox.origin()(2)*2 <<endl;

//        cout<<"test::initializeParts root2x = "<< GSHOTPyramid::TensorMap(root2x)() <<endl;

        mixture.initializeParts( nbParts, chairPartSize, root2x);

        for(int i=0; i < nbParts; ++i){
            cout<<"test::initializeParts offset["<<i+1<<"] = "<< mixture.models()[0].parts()[i+1].offset <<endl;
        }

        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval,
                      nbIterations, nbDatamine, maxNegSample);

//        Eigen::Vector3i origin(chairBox.origin()(0), chairBox.origin()(1), chairBox.origin()(2));//check comment : Mix:PosLatentSearch found a positive sample at : -2 -1 4 / 0.169058

//        Rectangle boxFound(origin, chairSize.first, chairSize.second, chairSize.third, sceneResolution*2);
//        viewer.displayCubeLine(boxFound, Eigen::Vector3i(255,0,255));

//        //Mix:PosLatentSearch found a positive sample with offsets : -2 -3 -3
//        Eigen::Vector3i partOrigin(origin(0) * 2 + mixture.models()[0].parts()[1].offset(0),
//                                   origin(1) * 2 + mixture.models()[0].parts()[1].offset(1),
//                                   origin(2) * 2 + mixture.models()[0].parts()[1].offset(2));
//        Rectangle partsBoxFound(partOrigin, chairPartSize.first, chairPartSize.second, chairPartSize.third, sceneResolution);

//        viewer.displayCubeLine(partsBoxFound, Eigen::Vector3i(0,255,0));



    }


    void detect(const Mixture & mixture, /*int depth, int height, int width, */int interval, const GSHOTPyramid & pyramid,
                double threshold, double overlap,/* const string image, */ostream & out,
                const string & images, vector<Detection> & detections, const Scene * scene = 0,
                Object::Name name = Object::UNKNOWN)
    {
        // Compute the scores
        vector<Tensor3DF> scores;
        vector<Mixture::Indices> argmaxes;
        vector<vector<vector<Model::Positions> > > positions;

        mixture.computeEnergyScores( pyramid, scores, argmaxes, &positions);

        // Cache the size of the models
        vector<Model::triple<int, int, int> > sizes(mixture.models().size());

        for (int i = 0; i < sizes.size(); ++i)
            sizes[i] = mixture.models()[i].rootSize();


        // For each scale
        for (int lvl = 0; lvl < scores.size(); ++lvl) {
//            const double scale = pow(2.0, static_cast<double>(lvl) / pyramid.interval() + 2);
            const double scale = 1 / pow(2.0, static_cast<double>(lvl) / interval);
            int offz = floor(scene->origin()(0)*scale);
            int offy = floor(scene->origin()(1)*scale);
            int offx = floor(scene->origin()(2)*scale);

            cout<<"test:: offz = "<<offz<<endl;
            cout<<"test:: offy = "<<offy<<endl;
            cout<<"test:: offx = "<<offx<<endl;

            const int depths = static_cast<int>(scores[lvl].depths());
            const int rows = static_cast<int>(scores[lvl].rows());
            const int cols = static_cast<int>(scores[lvl].cols());

            cout<<"test:: for lvl "<<lvl<<" :"<<endl;

            cout<<"test:: scores[lvl].depths() = "<<depths<<endl;
            cout<<"test:: scores[lvl].rows() = "<<rows<<endl;
            cout<<"test:: scores[lvl].cols() = "<<cols<<endl;

            if(scores[lvl].size() > 0){
                ofstream out("conv.txt");
                out << scores[lvl]();
            }

            cout << "test::detect limit z : " << offz << " / " << offz+depths<< endl;
            cout << "test::detect limit y : " << offy << " / " << offy+rows<< endl;
            cout << "test::detect limit x : " << offx << " / " << offx+cols<< endl;

            for (int z = 0; z < depths; ++z) {
                for (int y = 0; y < rows; ++y) {
                    for (int x = 0; x < cols; ++x) {
                        const double score = scores[lvl]()(z, y, x);

                        cout<<"test:: scores = "<<score<<endl;

                        ///////TO remove
//                        Eigen::Vector3i origin2((z+offz)/*- pad.z()*/,
//                                               (y+offy)/*- pad.y()*/,
//                                               (x+offx)/* - pad.x()*/);
//                        int w2 = sizes[argmaxes[lvl]()(z, y, x)].third /** scale*/;
//                        int h2 = sizes[argmaxes[lvl]()(z, y, x)].second /** scale*/;
//                        int d2 = sizes[argmaxes[lvl]()(z, y, x)].first /** scale*/;

//                        Rectangle bndbox2( origin2, d2, h2, w2, pyramid.resolutions()[lvl]);//indices of the cube in the PC

//                        cout<<"test:: detection bndbox = "<<bndbox2<<endl;
                        ////////////

                        //TODO !!!!!
                        if (score > threshold) {

                                Eigen::Vector3i origin((z+offz)/*- pad.z()*/,
                                                       (y+offy)/*- pad.y()*/,
                                                       (x+offx)/* - pad.x()*/);
                                int w = sizes[argmaxes[lvl]()(z, y, x)].third /** scale*/;
                                int h = sizes[argmaxes[lvl]()(z, y, x)].second /** scale*/;
                                int d = sizes[argmaxes[lvl]()(z, y, x)].first /** scale*/;

                                Rectangle bndbox( origin, d, h, w, pyramid.resolutions()[lvl]);//indices of the cube in the PC

                                cout<<"test:: detection bndbox = "<<bndbox<<endl;

                                if (!bndbox.empty()){
                                    detections.push_back(Detection(score, z, y, x, lvl, bndbox));
                                    cout<<"test:: bndbox added to detections"<<endl;
                                }

                        }
                    }
                }
            }
        }

        cout<<"test:: detections.size = "<<detections.size()<<endl;
        // Non maxima suppression
        sort(detections.begin(), detections.end());

        for (int i = 1; i < detections.size(); ++i){
            detections.resize(remove_if(detections.begin() + i, detections.end(),
                                        Intersector(detections[i - 1], overlap, true)) - detections.begin());
        }

        cout<<"test:: detections.size after intersection = "<<detections.size()<<endl;

        // Draw the detections
        int nb = 4;
        if (detections.size() > nb) {

            for (int i = 0; i < nb/*detections.size()*/; ++i) {

                const int x = detections[i].x;
                const int y = detections[i].y;
                const int z = detections[i].z;
                const int lvl = detections[i].lvl;

                const int argmax = argmaxes[lvl]()(z, y, x);

                cout<<"test:: argmax = "<<argmax<<endl;
                cout<<"test:: detection score = "<<detections[i].score<<endl;


                //draw each parts
                for (int j = 0; j < positions[argmax].size(); ++j) {
                    cout<<"test:: zp = "<<positions[argmax][j][lvl]()(z, y, x)(0)<<endl;
                    cout<<"test:: yp = "<<positions[argmax][j][lvl]()(z, y, x)(1)<<endl;
                    cout<<"test:: xp = "<<positions[argmax][j][lvl]()(z, y, x)(2)<<endl;
                    cout<<"test:: lvlp = "<<positions[argmax][j][lvl]()(z, y, x)(3)<<endl;

                    const int zp = positions[argmax][j][lvl]()(z, y, x)(0);
                    const int yp = positions[argmax][j][lvl]()(z, y, x)(1);
                    const int xp = positions[argmax][j][lvl]()(z, y, x)(2);
                    const int lvlp = positions[argmax][j][lvl]()(z, y, x)(3);

//                    cout<<"test:: positions[argmax][j][lvl]()(z, y, x) = "<<positions[argmax][j][lvl]()(z, y, x)<<endl;


//                    const double scale = pow(2.0, static_cast<double>(zp) / pyramid.interval() + 2);
//                    const double scale = 1 / pow(2.0, static_cast<double>(lvlp) / interval);


                    Eigen::Vector3i origin((zp+sceneBox.origin()(0)/* + detections[i].origin()(0)*2*/)/*- pad.z()*/,
                                           (yp+sceneBox.origin()(1) /*+ detections[i].origin()(1)*2*/)/*- pad.y()*/,
                                           (xp+sceneBox.origin()(2) /*+ detections[i].origin()(2)*2*/)/* - pad.x()*/);
                    int w = mixture.models()[argmax].partSize().third /** scale*/;
                    int h = mixture.models()[argmax].partSize().second /** scale*/;
                    int d = mixture.models()[argmax].partSize().first /** scale*/;

                    Rectangle bndbox( origin, d, h, w, pyramid.resolutions()[lvlp]);//indices of the cube in the PC

                    cout<<"test:: part bndbox to draw = "<<bndbox<<endl;

                    viewer.displayCubeLine(bndbox, Eigen::Vector3f(0,0,0), Vector3i(0,0,255));
                }

                // Draw the root last
                cout<<"test:: root bndbox = "<<detections[i]<<endl;
                Rectangle box(Vector3i(detections[i].origin()(0), detections[i].origin()(1), detections[i].origin()(2)),
                              mixture.models()[argmax].rootSize().first,
                              mixture.models()[argmax].rootSize().second,
                              mixture.models()[argmax].rootSize().third, pyramid.resolutions()[1]);
                viewer.displayCubeLine(box, Eigen::Vector3f(pyramid.resolutions()[0],pyramid.resolutions()[0],pyramid.resolutions()[0]),
                        Vector3i(0,255,255));
            }

        }
        if( detections.size() > 0){
            cout<<"test:: root bndbox = "<<detections[0]<<" with score : "<<detections[0].score<<endl;
            Rectangle box(Vector3i(detections[0].origin()(0), detections[0].origin()(1), detections[0].origin()(2)),
                          mixture.models()[0].rootSize().first,
                          mixture.models()[0].rootSize().second,
                          mixture.models()[0].rootSize().third, pyramid.resolutions()[1]);
            viewer.displayCubeLine(box,
                                   Eigen::Vector3f(pyramid.resolutions()[0],pyramid.resolutions()[0],pyramid.resolutions()[0]),
                    Vector3i(0,255,0));
        }
    }


    void testTest(){


        ifstream in("tmp.txt");

        if (!in.is_open()) {
            cerr << "Cannot open model file\n" << endl;
            return;
        }

        Mixture mixture;
        in >> mixture;
//        mixture.train_ = false;//allow quentin's code

        if (mixture.empty()) {
            cerr << "Invalid model file\n" << endl;
            return;
        }

        cout<<"test model root norm : "<<sqrt(GSHOTPyramid::TensorMap(mixture.models()[0].parts()[0].filter).squaredNorm())<<endl;



        int interval = 1;
        float threshold=0.005, overlap=0.5;
        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3), interval);

//        viewer.addPC(pyramid.keypoints_[0], 1, Eigen::Vector3i(100, 100, 100));
//        viewer.addPC(pyramid.keypoints_[1], 1, Eigen::Vector3i(255, 255, 255));

        ofstream out("tmpTest.txt");
        string images = sceneName;
        vector<Detection> detections;
        Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, chairBox);
        Scene scene( originScene, sceneBox.depth(), sceneBox.height(), sceneBox.width(), sceneName, {obj});

        detect(mixture, /*0, image.width(), image.height()*/interval, pyramid, threshold, overlap, /*file, */out,
               images, detections, &scene, Object::CHAIR);

        Rectangle rootBox(Vector3i(mixture.models()[0].parts()[0].offset(0),
                mixture.models()[0].parts()[0].offset(1),
                mixture.models()[0].parts()[0].offset(2)),
                      mixture.models()[0].rootSize().first, mixture.models()[0].rootSize().second,
                      mixture.models()[0].rootSize().third, pyramid.resolutions()[1]);
        viewer.displayCubeLine(rootBox,
                               Eigen::Vector3f(pyramid.resolutions()[0],pyramid.resolutions()[0],pyramid.resolutions()[0]),
                Vector3i(255,255,0));
        for(int i=1;i<mixture.models()[0].parts().size();++i){
            Rectangle partBox(Vector3i(mixture.models()[0].parts()[i].offset(0), mixture.models()[0].parts()[i].offset(1),
                    mixture.models()[0].parts()[i].offset(2)),
                          mixture.models()[0].partSize().first, mixture.models()[0].partSize().second,
                          mixture.models()[0].partSize().third, pyramid.resolutions()[0]);
            viewer.displayCubeLine(partBox,
                                   Eigen::Vector3f(0,0,0),
                    Vector3i(255,0,0));
        }

    }

    void testTrainSVM(){

//        Model::triple<int, int, int> rootSize( sceneSize.first/4,
//                                           sceneSize.second/4,
//                                               sceneSize.third/4);
//        Model::triple<int, int, int> partSize( sceneSize.first/6,
//                                               sceneSize.second/6,
//                                               sceneSize.third/6);
//        Rectangle sceneRec( Eigen::Vector3i(1, 1, 1), rootSize.first, rootSize.second, rootSize.third, resolution);
//        Rectangle sceneRec2( Eigen::Vector3i(3, 5, 4), rootSize.first, rootSize.second, rootSize.third, resolution);
////        Rectangle sceneRec3( Eigen::Vector3i(2, 2, 2), rootSize.third*0.75, rootSize.second*2, rootSize.first);

//        cout<<"test::sceneRec : "<< sceneRec <<endl;
//        cout<<"test::sceneRec2 : "<< sceneRec2 <<endl;
//        cout<<"test::rootSize : "<<rootSize.first<<" "<<rootSize.second<<" "<<rootSize.third<<endl;
//        cout<<"test::partSize : "<<partSize.first<<" "<<partSize.second<<" "<<partSize.third<<endl;

//        vector<pair<Model, int> >  positives, negatives;
//        Model modelScene( rootSize, 1, partSize);
//        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3));

//        modelScene.parts()[0].filter = pyramid.levels()[0].block( 1, 1, 1, rootSize.first, rootSize.second, rootSize.third);
//        modelScene.parts()[1].filter = pyramid.levels()[0].block( 2, 2, 1, partSize.first, partSize.second, partSize.third);

//        positives.push_back(pair<Model, int>(modelScene, 0));

//        modelScene.parts()[0].filter = pyramid.levels()[0].block( 5, 5, 5, rootSize.first, rootSize.second, rootSize.third);
//        modelScene.parts()[1].filter = pyramid.levels()[0].block( 5, 5, 5, partSize.first, partSize.second, partSize.third);

//        negatives.push_back(pair<Model, int>(modelScene, 0));


//        Model model( rootSize, 1, partSize);
//        std::vector<Model> models = { model};

//        Mixture mixture( models);

//        double C = 0.002;
//        double J = 2.0;
//        mixture.trainSVM(positives, negatives, C, J);

//        ofstream out("tmp.txt");

//        out << (mixture);

//        ////////////

//        modelScene.parts()[0].filter = pyramid.levels()[0].block( 5, 1, 1, rootSize.first, rootSize.second, rootSize.third);
//        modelScene.parts()[1].filter = pyramid.levels()[0].block( 5, 2, 1, partSize.first, partSize.second, partSize.third);

//        positives[0] = pair<Model, int>(modelScene, 0);

//        modelScene.parts()[0].filter = pyramid.levels()[0].block( 0, 0, 5, rootSize.first, rootSize.second, rootSize.third);
//        modelScene.parts()[1].filter = pyramid.levels()[0].block( 1, 1, 5, partSize.first, partSize.second, partSize.third);

//        negatives[0] = pair<Model, int>(modelScene, 0);

//        mixture.trainSVM(positives, negatives, C, J);

//        ofstream out2("tmp2.txt");

//        out2 << (mixture);
    }

    char* sceneName;
    char* tableName;
    PointCloudPtr sceneCloud;
    PointCloudPtr chairCloud;
    PointCloudPtr tableCloud;
    Vector3i originScene;
    Rectangle sceneBox;
    Rectangle chairBox;
    Rectangle tableBox;
    float starting_resolution;
    float sceneResolution;
    Viewer viewer;
};


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


    Test test( "/home/ubuntu/3DDataset/3DDPM/testSceneMiddle_compress.pcd", "/home/ubuntu/3DDataset/3DDPM/chair.pcd", "/home/ubuntu/3DDataset/3DDPM/table.pcd");

    // testSceneMiddle_compress.pcd
    // smallScene2.pcd
    int start = getMilliCount();

//    test.testTrainSVM();//OK
//    test.testPosLatSearch();
//    test.testNegLatSearch();

//    test.createChairModel();

//    test.initSample();

//    test.testTrain();

    test.testTest();




    int end = getMilliCount();

    cout << "Time : " << end-start << endl;


    test.viewer.show();




    return 0;
}

