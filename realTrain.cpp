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
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

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

        sceneResolution = 0.09/2.0;
        cout<<"test::sceneResolution : "<<sceneResolution<<endl;

    }

    void train(string positiveFolder, string negativeFolder){

        int nbParts = 4;
        double C = 0.002, J = 2;
        int interval = 1, nbIterations = 1, nbDatamine = 1, maxNegSample = 20;
        Model::triple<int, int, int> chairSize(8,10,6);//5,6,4in lvl 1
        Model::triple<int, int, int> chairPartSize(8,10,6);//in lvl 0
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

        vector<Scene> scenes(positiveListFiles.size() + negativeListFiles.size());

        for( int i=0; i < positiveListFiles.size(); ++i){
            printf ("%s\n", positiveListFiles[i].c_str());
            PointCloudPtr cloud( new PointCloudT);

            if (readPointCloud(positiveListFiles[i], cloud) == -1) {
                cout<<"couldnt open ply file"<<endl;
            }

            PointType min;
            PointType max;
            pcl::getMinMax3D(*cloud, min, max);

            Vector3i resolution( (max.z - min.z)/sceneResolution+1,
                                 (max.y - min.y)/sceneResolution+1,
                                 (max.x - min.x)/sceneResolution+1);
            cout << "scene resolution : " << resolution << endl;
            Vector3i originScene = Vector3i(floor(min.z/sceneResolution),
                                   floor(min.y/sceneResolution),
                                   floor(min.x/sceneResolution));
            Vector3i originChair = Vector3i(floor(min.z/sceneResolution/2.0),
                                   floor(min.y/sceneResolution/2.0),
                                   floor(min.x/sceneResolution/2.0));
            Model::triple<int, int, int> sceneSize( resolution(0)/2.0, resolution(1)/2.0, resolution(2)/2.0);//5,6,4);

            cout << "scene min : " << min << endl;


            Rectangle chairBox(originChair , sceneSize.first, sceneSize.second, sceneSize.third, sceneResolution*2/*resolution.sum()/3.0*/);

            cout << "chairBox : " << chairBox << endl;
            Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, chairBox);
            scenes[i] = Scene( originScene, chairBox.depth(), chairBox.height(), chairBox.width(),
                               positiveListFiles[i], {obj});
        }


        for( int i=0; i < negativeListFiles.size(); ++i){
            printf ("%s\n", negativeListFiles[i].c_str());
            PointCloudPtr cloud( new PointCloudT);

            if (readPointCloud(negativeListFiles[i], cloud) == -1) {
                cout<<"couldnt open ply file"<<endl;
            }

            PointType min;
            PointType max;
            pcl::getMinMax3D(*cloud, min, max);

            Vector3i resolution( (max.z - min.z)/sceneResolution+1,
                                 (max.y - min.y)/sceneResolution+1,
                                 (max.x - min.x)/sceneResolution+1);
            cout << "scene resolution : " << resolution << endl;

            Vector3i originScene = Vector3i(floor(min.z/sceneResolution),
                                   floor(min.y/sceneResolution),
                                   floor(min.x/sceneResolution));
            Vector3i originBox = Vector3i(floor(min.z/sceneResolution/2.0),
                                   floor(min.y/sceneResolution/2.0),
                                   floor(min.x/sceneResolution/2.0));
            Model::triple<int, int, int> sceneSize( resolution(0)/2.0, resolution(1)/2.0, resolution(2)/2.0);//5,6,4);

            cout << "scene min : " << min << endl;

            Rectangle box(originBox , sceneSize.first, sceneSize.second, sceneSize.third, sceneResolution*2);

            Object obj(Object::BIRD, Object::Pose::UNSPECIFIED, false, false, box);
            scenes[positiveListFiles.size() + i] = Scene( originScene, box.depth(), box.height(), box.width(),
                               negativeListFiles[i], {obj});
        }


        Mixture mixture( models);

        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, nbIterations/nbIterations,
                      nbDatamine, maxNegSample, C, J);

        //TODO include initializeParts in train()
        mixture.initializeParts( nbParts, chairPartSize/*, root2x*/);

//        for(int i=0; i < nbParts; ++i){
//            cout<<"test::initializeParts offset["<<i+1<<"] = "<< mixture.models()[0].parts()[i+1].offset <<endl;
//        }

        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval,
                      nbIterations, nbDatamine, maxNegSample, C, J);

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

            float maxi = scores[lvl].max();
            cout << "test::detect limit y : " << offy << " / " << offy+rows<< endl;

            PointCloudPtr scoreCloud(new PointCloudT());
            scoreCloud->width    = scores[lvl].size();
            scoreCloud->height   = 1;
            scoreCloud->points.resize (scoreCloud->width * scoreCloud->height);
            cout << "test::detect limit x : " << offx << " / " << offx+cols<< endl;

            for (int z = 0; z < depths; ++z) {
                for (int y = 0; y < rows; ++y) {
                    for (int x = 0; x < cols; ++x) {
                        const double score = scores[lvl]()(z, y, x);

                        PointType p = PointType();
                        p.z = pyramid.keypoints_[1]->at(x + y * cols + z * rows * cols).z;
                        p.y = pyramid.keypoints_[1]->at(x + y * cols + z * rows * cols).y;
                        p.x = pyramid.keypoints_[1]->at(x + y * cols + z * rows * cols).x;
                        p.r = score / maxi*255;
                        p.g = score / maxi*255;
                        p.b = score / maxi*255;
                        scoreCloud->at(x + y * cols + z * rows * cols) = p;

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
//            viewer.addPC(scoreCloud, 4, Eigen::Vector3i(255, 255, 255));

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

//                cout<<"test:: argmax = "<<argmax<<endl;
//                cout<<"test:: detection score = "<<detections[i].score<<endl;


                //draw each parts
                for (int j = 0; j < positions[argmax].size(); ++j) {
//                    cout<<"test:: zp = "<<positions[argmax][j][lvl]()(z, y, x)(0)<<endl;
//                    cout<<"test:: yp = "<<positions[argmax][j][lvl]()(z, y, x)(1)<<endl;
//                    cout<<"test:: xp = "<<positions[argmax][j][lvl]()(z, y, x)(2)<<endl;
//                    cout<<"test:: lvlp = "<<positions[argmax][j][lvl]()(z, y, x)(3)<<endl;

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

//                    cout<<"test:: part bndbox to draw = "<<bndbox<<endl;

                    viewer.displayCubeLine(bndbox, Eigen::Vector3f(0,0,0), Vector3i(0,0,255));
                }

                // Draw the root last
//                cout<<"test:: root bndbox = "<<detections[i]<<endl;
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


        ifstream in("tmp.txt");

        if (!in.is_open()) {
            cerr << "Cannot open model file\n" << endl;
            return;
        }

        Mixture mixture;
        in >> mixture;
        mixture.train_ = false;//allow quentin's code

        if (mixture.empty()) {
            cerr << "Invalid model file\n" << endl;
            return;
        }



        int interval = 1;
        float threshold=0., overlap=0.5;
        PointCloudPtr cloud( new PointCloudT);
        if (readPointCloud(sceneName, cloud) == -1) {
            cout<<"test::couldnt open pcd file"<<endl;
        }

        PointType minTmp;
        PointType maxTmp;
        pcl::getMinMax3D(*cloud , minTmp, maxTmp);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

          // Define a translation of 2.5 meters on the x axis.
//          transform.translation() << -minTmp.x, -minTmp.y, -minTmp.z;

          // The same rotation matrix as before; theta radians around Z axis
//          transform.rotate (Eigen::AngleAxisf (-1.57, Eigen::Vector3f::UnitX()));
//          transform.rotate (Eigen::AngleAxisf (/*-2**/1.57, Eigen::Vector3f::UnitY()));//for smallScene3
//          transform.rotate (Eigen::AngleAxisf (1.57, Eigen::Vector3f::UnitZ()));

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

//        Rectangle rootBox(Vector3i(mixture.models()[0].parts()[0].offset(0),
//                mixture.models()[0].parts()[0].offset(1),
//                mixture.models()[0].parts()[0].offset(2)),
//                      mixture.models()[0].rootSize().first, mixture.models()[0].rootSize().second,
//                      mixture.models()[0].rootSize().third, pyramid.resolutions()[1]);
//        viewer.displayCubeLine(rootBox,
//                               Eigen::Vector3f(pyramid.resolutions()[0],pyramid.resolutions()[0],pyramid.resolutions()[0]),
//                Vector3i(255,255,0));
//        for(int i=1;i<mixture.models()[0].parts().size();++i){
//            Rectangle partBox(Vector3i(mixture.models()[0].parts()[i].offset(0), mixture.models()[0].parts()[i].offset(1),
//                    mixture.models()[0].parts()[i].offset(2)),
//                          mixture.models()[0].partSize().first, mixture.models()[0].partSize().second,
//                          mixture.models()[0].partSize().third, pyramid.resolutions()[0]);
//            viewer.displayCubeLine(partBox,
//                                   Eigen::Vector3f(0,0,0),
//                    Vector3i(255,0,0));
//        }

    }

    void checkImages(string positiveFolder){
        vector<string> positiveListFiles;
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

        ifstream in("tmp.txt");

        if (!in.is_open()) {
            cerr << "Cannot open model file\n" << endl;
            return;
        }

        Mixture mixture;
        in >> mixture;

        if (mixture.empty()) {
            cerr << "Invalid model file\n" << endl;
            return;
        }


        int nbParts = 4 + 1;
        Mat img( positiveListFiles.size() + 1, nbParts * GSHOTPyramid::DescriptorSize, CV_8UC3, cv::Scalar(0,0,0));

        int interval = 1;

        for(int y=0;y<positiveListFiles.size();y++){
            float maxi = 0;

            PointCloudPtr cloud( new PointCloudT);
            if (readPointCloud(positiveListFiles[y], cloud) == -1) {
                cout<<"test::couldnt open pcd file"<<endl;
            }
            GSHOTPyramid pyramid(cloud, Eigen::Vector3i( 3,3,3), interval);

            GSHOTPyramid::Level lvl = pyramid.levels()[1].agglomerate();


            for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
                if( maxi < lvl()(0,0,0)(x)) maxi = lvl()(0,0,0)(x);
            }
            cout<<"maxi descriptor value = "<<maxi<<endl;


            for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
                Vec3b color = img.at<Vec3b>(Point(x,y));
                color[0] = lvl()(0,0,0)(x)/maxi*255/*pyramid.levels()[1].size()*/;
                color[1] = color[0];
                color[2] = color[0];
                img.at<Vec3b>(Point(x,y)) = color;
            }

            for(int p=1;p<nbParts;p++){
                maxi = 0;
                GSHOTPyramid::Level lvl = pyramid.levels()[0].agglomerateBlock(
                        mixture.models()[0].parts()[p].offset(0),
                        mixture.models()[0].parts()[p].offset(1),
                        mixture.models()[0].parts()[p].offset(2),
                        mixture.models()[0].partSize().first,
                        mixture.models()[0].partSize().second,
                        mixture.models()[0].partSize().third);

                for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
                    if( maxi < lvl()(0,0,0)(x)) maxi = lvl()(0,0,0)(x);
                }
                cout<<"maxi descriptor value = "<<maxi<<endl;

                for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
                    Vec3b color = img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,y));
                    color[0] = lvl()(0,0,0)(x)/maxi*255/*/mixture.models()[0].parts()[p].filter.size()*/;
                    color[1] = color[0];
                    color[2] = color[0];
                    img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,y)) = color;
                }

            }

        }


        for(int p=0;p<nbParts;p++){

            GSHOTPyramid::Level mod = mixture.models()[0].parts()[p].filter.agglomerate();

            float maxi = 0, mini = mod()(0,0,0)(0);
            for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
                if( maxi < mod()(0,0,0)(x)) maxi = mod()(0,0,0)(x);
                if( mini > mod()(0,0,0)(x)) mini = mod()(0,0,0)(x);
            }
            cout<<"maxi model descriptor value = "<<maxi<<endl;
            cout<<"mini model descriptor value = "<<mini<<endl;

            for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
                if(mod()(0,0,0)(x) > 0){
                    Vec3b color = img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,positiveListFiles.size()));
                    color[2] = mod()(0,0,0)(x)/maxi*255;//*mixture.models()[0].parts()[p].filter.size()*/;
                    img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,positiveListFiles.size())) = color;
                } else{
                    Vec3b color = img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,positiveListFiles.size()));
                    color[1] = mod()(0,0,0)(x)/mini*255;//*mixture.models()[0].parts()[p].filter.size()*/;
                    img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,positiveListFiles.size())) = color;
                }
            }

        }

        if (img.empty()){
            cout << "\n Image not created. You"
                         " have done something wrong. \n";
            return;    // Unsuccessful.
        }

        imwrite( "img.jpg", img );

        namedWindow("A_good_name", CV_WINDOW_AUTOSIZE);

        imshow("A_good_name", img);
//        resizeWindow("A_good_name", 600,600);

        waitKey(0); //wait infinite time for a keypress

        destroyWindow("A_good_name");
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


//    test.train("/home/ubuntu/3DDataset/3DDPM/chair_normalized/",
//               "/media/ubuntu/DATA/3DDataset/Cat51_normalized/monster_truck/full/");

    test.test( "/home/ubuntu/3DDataset/3DDPM/smallScene3.pcd");

    //1 block over 2 is black, why ????
//    test.checkImages("/home/ubuntu/3DDataset/3DDPM/table/");

    // testSceneMiddle_compress.pcd
    // smallScene2.pcd
    // scene_2.ply

    int end = getMilliCount();

    cout << "Time : " << end-start << endl;

    ifstream in("tableModel4parts1pos.txt");

    if (!in.is_open()) {
        cerr << "Cannot open model file\n" << endl;
    }

    Mixture mixTable;
    in >> mixTable;

    if (mixTable.empty()) {
        cerr << "Invalid model file\n" << endl;
    }

    ifstream in2("chairModel4parts1pos.txt");

    if (!in2.is_open()) {
        cerr << "Cannot open model file\n" << endl;
    }

    Mixture mixChair;
    in2 >> mixChair;

    if (mixChair.empty()) {
        cerr << "Invalid model file\n" << endl;
    }

    cout<<"Chair dot Table = " << mixChair.models()[0].dot(mixTable.models()[0]) << endl;
    cout<<"Table dot Chair = " << mixTable.models()[0].dot(mixChair.models()[0]) << endl;
    cout<<"Table dot Table = " << mixTable.models()[0].dot(mixTable.models()[0]) << endl;
    cout<<"Chair dot Chair = " << mixChair.models()[0].dot(mixChair.models()[0]) << endl;

    cout<<"Chair norm = " << mixChair.models()[0].norm() << endl;
    cout<<"Table norm = " << mixTable.models()[0].norm() << endl;

    cout<<"Part 0 agglo : " << endl;

    cout<<"Chair dot Table = " << mixChair.models()[0].parts()[0].filter.agglomerate().dot(mixTable.models()[0].parts()[0].filter.agglomerate()) << endl;
    cout<<"Table dot Chair = " << mixTable.models()[0].parts()[0].filter.agglomerate().dot(mixChair.models()[0].parts()[0].filter.agglomerate()) << endl;
    cout<<"Table dot Table = " << mixTable.models()[0].parts()[0].filter.agglomerate().dot(mixTable.models()[0].parts()[0].filter.agglomerate()) << endl;
    cout<<"Chair dot Chair = " << mixChair.models()[0].parts()[0].filter.agglomerate().dot(mixChair.models()[0].parts()[0].filter.agglomerate()) << endl;



//    Mat img( GSHOTPyramid::DescriptorSize, GSHOTPyramid::DescriptorSize, CV_8UC3, cv::Scalar(0,0,0));

//    PointCloudPtr cloud( new PointCloudT);
//    if (readPointCloud("/home/ubuntu/3DDataset/3DDPM/table.pcd", cloud) == -1) {
//        cout<<"test::couldnt open pcd file"<<endl;
//    }

//    PointType minTmp;
//    PointType min;
//    PointType max;
//    pcl::getMinMax3D(*cloud, minTmp, max);

//    min.x = floor(minTmp.x/0.09)*0.09;
//    min.y = floor(minTmp.y/0.09)*0.09;
//    min.z = floor(minTmp.z/0.09)*0.09;

//    float resolution = 0.009;
//    PointCloudPtr keypoints = GSHOTPyramid::compute_keypoints(resolution, min, max, 0);

//    DescriptorsPtr descriptors (new Descriptors());
//    SurfaceNormalsPtr normals (new SurfaceNormals());

//    pcl::NormalEstimation<PointType,NormalType> norm_est;
//    norm_est.setKSearch (8);
//    norm_est.setInputCloud (cloud);
//    norm_est.compute (*normals);

//    pcl::SHOTEstimation<PointType, NormalType, DescriptorType> descr_est;
//    descr_est.setRadiusSearch (2*resolution);
//    descr_est.setInputCloud (keypoints);
//    descr_est.setInputNormals (normals);
//    descr_est.setSearchSurface (cloud);
//    descr_est.compute (*descriptors);

//    cout<<"GSHOT:: descriptors size = "<<descriptors->size()<<endl;


//    for (size_t i = 0; i < descriptors->size(); ++i){

//        if (pcl_isnan(descriptors->points[i].descriptor[0])){
//            descriptors->points[i].descriptor[0] = 0;
//        }
//        for (size_t j = 0; j < GSHOTPyramid::DescriptorSize; ++j){

//            if (pcl_isnan(descriptors->points[i].descriptor[j])){
//                descriptors->points[i].descriptor[j] = 0;
//            }
//        }
//    }


//    GSHOTPyramid::Level level( descriptors->size(),1,1);
//    int kpt = 0;
////#pragma omp parallel for
//    for (int z = 0; z < level.depths(); ++z){
//        for (int y = 0; y < level.rows(); ++y){
//            for (int x = 0; x < level.cols(); ++x){
//                for( int k = 0; k < GSHOTPyramid::DescriptorSize; ++k){
//                    level()(z, y, x)(k) = descriptors->points[kpt].descriptor[k];
////                            if(descriptors->points[kpt].descriptor[k]<0){
////                                cout << "GSHOTPyr::constructor descriptors->points["<<kpt<<"].descriptor["<<k<<"] = "
////                                     << descriptors->points[kpt].descriptor[k] << endl;
////                            }
//                }
//                ++kpt;
//            }
//        }
//    }

//    GSHOTPyramid::Level lvl = level.agglomerate();

//    float maxi = 0, mini = lvl()(0,0,0)(0);
//    for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
//        if( maxi < lvl()(0,0,0)(x)) maxi = lvl()(0,0,0)(x);
//        if( mini > lvl()(0,0,0)(x)) mini = lvl()(0,0,0)(x);
//    }
//    cout<<"maxi model descriptor value = "<<maxi<<endl;
//    cout<<"mini model descriptor value = "<<mini<<endl;


//    for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
//        for(int y=0;y<GSHOTPyramid::DescriptorSize;y++){

//            Vec3b color = img.at<Vec3b>(Point(x, y));
//            color[0] = lvl()(0,0,0)(x)/maxi*255;//*mixture.models()[0].parts()[p].filter.size()*/;
//            color[1] = color[0];
//            color[2] = color[0];
//            img.at<Vec3b>(Point(x, y)) = color;
//        }
//    }



//    if (img.empty()){
//        cout << "\n Image not created. You"
//                     " have done something wrong. \n";
//        return 0;    // Unsuccessful.
//    }

//    imwrite( "img.jpg", img );

//    namedWindow("A_good_name", CV_WINDOW_AUTOSIZE);

//    imshow("A_good_name", img);
////        resizeWindow("A_good_name", 600,600);

//    waitKey(0); //wait infinite time for a keypress

//    destroyWindow("A_good_name");


    test.viewer.show();




    return 0;
}

