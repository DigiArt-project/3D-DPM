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

    void train(string positiveFolder, string negativeFolder){

        int nbParts = 6;
        Model::triple<int, int, int> chairSize(6,4,5);//in lvl 1
        Model::triple<int, int, int> chairPartSize( 6,4,5);//in lvl 0
        Model model( chairSize, 0);
        std::vector<Model> models = { model};

        vector<string> positiveListFiles, negativeListFiles;
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (positiveFolder.c_str())) != NULL) {
          while ((ent = readdir (dir)) != NULL) {
              if( string(ent->d_name).compare(".") != 0 && string(ent->d_name).compare("..") != 0)
                positiveListFiles.push_back( positiveFolder + ent->d_name);
//            printf ("%s\n", (folder + ent->d_name).c_str());
          }
          closedir (dir);
        } else {
          perror ("could not open directory");
        }

        if ((dir = opendir (negativeFolder.c_str())) != NULL) {
          while ((ent = readdir (dir)) != NULL) {
              if( string(ent->d_name).compare(".") != 0 && string(ent->d_name).compare("..") != 0)
                negativeListFiles.push_back( negativeFolder + ent->d_name);
//            printf ("%s\n", (folder + ent->d_name).c_str());
          }
          closedir (dir);
        } else {
          perror ("could not open directory");
        }

        vector<Scene> scenes(positiveListFiles.size()/* + negativeListFiles.size()*/);

        for( int i=0; i < positiveListFiles.size(); ++i){
            printf ("%s\n", positiveListFiles[i].c_str());
            PointCloudPtr cloud( new PointCloudT);

            if (pcl::io::loadPLYFile<PointType>(positiveListFiles[i].c_str(), *cloud) == -1) {
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
            Model::triple<int, int, int> sceneSize( ceil((max.z-originScene(0)*resolution(0))/resolution(0))+1,
                                                    ceil((max.y-originScene(1)*resolution(1))/resolution(1))+1,
                                                    ceil((max.x-originScene(2)*resolution(2))/resolution(2))+1);
            Rectangle chairBox(originScene , sceneSize.first, sceneSize.second, sceneSize.third, sceneResolution);

            cout << "chairBox : " << chairBox << endl;
            Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, chairBox);
            scenes[i] = Scene( originScene, chairBox.depth(), chairBox.height(), chairBox.width(),
                               positiveListFiles[i], {obj});
        }


//        for( int i=0; i < negativeListFiles.size(); ++i){
//            printf ("%s\n", negativeListFiles[i].c_str());
//            PointCloudPtr cloud( new PointCloudT);

//            if (pcl::io::loadPLYFile<PointType>(negativeListFiles[i].c_str(), *cloud) == -1) {
//                cout<<"couldnt open ply file"<<endl;
//            }

//            PointType min;
//            PointType max;
//            pcl::getMinMax3D(*cloud, min, max);

//            Vector3f resolution( (max.z - min.z)/(chairSize.first),
//                                 (max.y - min.y)/(chairSize.second),
//                                 (max.x - min.x)/(chairSize.third));

//            Vector3i originScene = Vector3i(floor(min.z/resolution(0)),
//                                   floor(min.y/resolution(1)),
//                                   floor(min.x/resolution(2)));
//            Model::triple<int, int, int> sceneSize( ceil((max.z-originScene(0)*resolution(0))/resolution(0))+1,
//                                                    ceil((max.y-originScene(1)*resolution(1))/resolution(1))+1,
//                                                    ceil((max.x-originScene(2)*resolution(2))/resolution(2))+1);
//            Rectangle box(originScene , sceneSize.first, sceneSize.second, sceneSize.third, sceneResolution);

//            Object obj(Object::BIRD, Object::Pose::UNSPECIFIED, false, false, box);
//            scenes[positiveListFiles.size() + i] = Scene( originScene, box.depth(), box.height(), box.width(),
//                               negativeListFiles[i], {obj});
//        }


        Mixture mixture( models);

        int interval = 1, nbIterations = 5, nbDatamine = 2, maxNegSample = 20;
        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, nbIterations/nbIterations,
                      nbDatamine, maxNegSample);

        //TODO include initializeParts in train()
        mixture.initializeParts( nbParts, chairPartSize/*, root2x*/);

//        for(int i=0; i < nbParts; ++i){
//            cout<<"test::initializeParts offset["<<i+1<<"] = "<< mixture.models()[0].parts()[i+1].offset <<endl;
//        }

        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval,
                      nbIterations, nbDatamine, maxNegSample);

    }


    void detect(const Mixture & mixture, /*int depth, int height, int width, */int interval, const GSHOTPyramid & pyramid,
                double threshold, double overlap,/* const string image, */ostream & out,
                const string & images, vector<Detection> & detections, const Scene * scene = 0,
                Object::Name name = Object::CHAIR)
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
        int nb = 6;
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


                    Eigen::Vector3i origin((zp+scene->origin()(0)/* + detections[i].origin()(0)*2*/)/*- pad.z()*/,
                                           (yp+scene->origin()(1) /*+ detections[i].origin()(1)*2*/)/*- pad.y()*/,
                                           (xp+scene->origin()(2) /*+ detections[i].origin()(2)*2*/)/* - pad.x()*/);
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


    void test( string sceneName){
        sceneResolution = 0.09;

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
        PointCloudPtr cloud( new PointCloudT);
        if (readPointCloud(sceneName, cloud) == -1) {
            cout<<"test::couldnt open pcd file"<<endl;
        }

        PointType minTmp;
        PointType maxTmp;
        pcl::getMinMax3D(*cloud , minTmp, maxTmp);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

          // Define a translation of 2.5 meters on the x axis.
          transform.translation() << -minTmp.x, -minTmp.y, -minTmp.z;

          // The same rotation matrix as before; theta radians around Z axis
          transform.rotate (Eigen::AngleAxisf (1.57, Eigen::Vector3f::UnitX()));
          PointCloudPtr sceneCloud (new PointCloudT);
            // You can either apply transform_1 or transform_2; they are the same
            pcl::transformPointCloud (*cloud, *sceneCloud, transform);
        GSHOTPyramid pyramid(sceneCloud, Eigen::Vector3i( 3,3,3), interval);

        PointType minScene;
        PointType maxScene;
        pcl::getMinMax3D(*sceneCloud , minScene, maxScene);

        Vector3i originScene( floor(minScene.z/sceneResolution),
                               floor(minScene.y/sceneResolution),
                               floor(minScene.x/sceneResolution));
        Model::triple<int, int, int> sceneSize( ceil((maxScene.z-originScene(0)*sceneResolution)/sceneResolution)+1,
                                                ceil((maxScene.y-originScene(1)*sceneResolution)/sceneResolution)+1,
                                                ceil((maxScene.x-originScene(2)*sceneResolution)/sceneResolution)+1);



        viewer.addPC(pyramid.keypoints_[1], 1, Eigen::Vector3i(255, 255, 255));

        ofstream out("tmpTest.txt");
        vector<Detection> detections;
        Scene scene( originScene, sceneSize.first, sceneSize.second, sceneSize.third, sceneName, {});

        detect(mixture, interval, pyramid, threshold, overlap, /*file, */out,
               sceneName, detections, &scene, Object::CHAIR);

        PointCloudPtr subspace(new PointCloudT());
        pcl::UniformSampling<PointType> sampling;
        sampling.setInputCloud(sceneCloud);
        sampling.setRadiusSearch (sceneResolution);
        sampling.filter(*subspace);

        viewer.addPC( sceneCloud, 3);

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


    test.train("/media/ubuntu/DATA/3DDataset/Cat31_normalized/chair/full/",
               "/media/ubuntu/DATA/3DDataset/StructureSensor_normalized/jar/full/");

    test.test( "/home/ubuntu/3DDataset/3DDPM/scene_2.ply");

    // testSceneMiddle_compress.pcd
    // smallScene2.pcd
    // scene_2.ply

    int end = getMilliCount();

    cout << "Time : " << end-start << endl;


    test.viewer.show();




    return 0;
}

