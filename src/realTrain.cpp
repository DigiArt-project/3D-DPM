// Written by Fisichella Thomas
// Date 25/05/2018

#include "Mixture.h"
#include "Intersector.h"
#include "Object.h"


#include <cstdlib>
#include <sys/timeb.h>

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

bool ascendingOrder( struct Detection score1, struct Detection score2){
    return score2 < score1;
}

class Test{
public:

    Test()
    {

        sceneResolution = 0.05;//0.09/2.0;
        cout<<"test::sceneResolution : "<<sceneResolution<<endl;

    }

    void oldTrain(string positiveFolder, string negativeFolder){

//        int nbParts = 3;
//        double C = 0.002, J = 2;
//        int interval = 1, nbIterations = 3, nbDatamine = 3, maxNegSample = 2000;
//        Model::triple<int, int, int> chairSize(8,10,6);//8,10,6 in lvl 1
//        Model::triple<int, int, int> chairPartSize(8,10,6);//8,10,6 in lvl 0
//        Model model( chairSize, 0);
//        std::vector<Model> models = { model};

//        vector<string> positiveListFiles, negativeListFiles;
//        DIR *dir;
//        struct dirent *ent;
//        if ((dir = opendir (positiveFolder.c_str())) != NULL) {
//          while ((ent = readdir (dir)) != NULL) {
//              if( string(ent->d_name).compare(".") != 0 && string(ent->d_name).compare("..") != 0)
//                positiveListFiles.push_back( positiveFolder + ent->d_name);
//    //            printf ("%s\n", (folder + ent->d_name).c_str());
//          }
//          closedir (dir);
//        } else {
//          perror ("could not open directory");
//        }

//        if ((dir = opendir (negativeFolder.c_str())) != NULL) {
//          while ((ent = readdir (dir)) != NULL) {
//              if( string(ent->d_name).compare(".") != 0 && string(ent->d_name).compare("..") != 0)
//    //                  if( string(ent->d_name).find(".ply") != std::string::npos)
//                    negativeListFiles.push_back( negativeFolder + ent->d_name);
//    //            printf ("%s\n", (folder + ent->d_name).c_str());
//          }
//          closedir (dir);
//        } else {
//          perror ("could not open directory");
//        }

//        vector<Scene> scenes(positiveListFiles.size() + negativeListFiles.size());

//        for( int i=0; i < positiveListFiles.size(); ++i){
//            printf ("%s\n", positiveListFiles[i].c_str());
//            PointCloudPtr cloud( new PointCloudT);

//            if (readPointCloud(positiveListFiles[i], cloud) == -1) {
//                cout<<"couldnt open ply file"<<endl;
//            }

//            PointType min;
//            PointType max;
//            pcl::getMinMax3D(*cloud, min, max);

//            Vector3i resolution( (max.z - min.z)/sceneResolution+1,
//                                 (max.y - min.y)/sceneResolution+1,
//                                 (max.x - min.x)/sceneResolution+1);
//            cout << "scene resolution : " << resolution << endl;
//            Vector3i originScene = Vector3i(floor(min.z/sceneResolution),
//                                   floor(min.y/sceneResolution),
//                                   floor(min.x/sceneResolution));
//            Vector3i originChair = Vector3i(floor(min.z/sceneResolution/2.0),
//                                   floor(min.y/sceneResolution/2.0),
//                                   floor(min.x/sceneResolution/2.0));
//            Model::triple<int, int, int> sceneSize = chairSize;//( resolution(0)/2.0, resolution(1)/2.0, resolution(2)/2.0);

//            cout << "scene min : " << min << endl;


//            Rectangle chairBox(originChair , sceneSize.first, sceneSize.second, sceneSize.third, sceneResolution*2/*resolution.sum()/3.0*/);

//            cout << "chairBox : " << chairBox << endl;
//            Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, chairBox);
//            scenes[i] = Scene( originScene, chairBox.depth(), chairBox.height(), chairBox.width(),
//                               positiveListFiles[i], {obj});
//        }


//        for( int i=0; i < negativeListFiles.size(); ++i){
//            printf ("%s\n", negativeListFiles[i].c_str());
//            PointCloudPtr cloud( new PointCloudT);

//            if (readPointCloud(negativeListFiles[i], cloud) == -1) {
//                cout<<"couldnt open ply file"<<endl;
//            }

//            PointType min;
//            PointType max;
//            pcl::getMinMax3D(*cloud, min, max);

//            Vector3i resolution( (max.z - min.z)/sceneResolution+1,
//                                 (max.y - min.y)/sceneResolution+1,
//                                 (max.x - min.x)/sceneResolution+1);
//            cout << "scene resolution : " << resolution << endl;

//            Vector3i originScene = Vector3i(floor(min.z/sceneResolution),
//                                   floor(min.y/sceneResolution),
//                                   floor(min.x/sceneResolution));
//            Vector3i originBox = Vector3i(floor(min.z/sceneResolution/2.0),
//                                   floor(min.y/sceneResolution/2.0),
//                                   floor(min.x/sceneResolution/2.0));
//            Model::triple<int, int, int> sceneSize = chairSize;//( resolution(0)/2.0, resolution(1)/2.0, resolution(2)/2.0);

//            cout << "negative scene min : " << min << endl;

//            Rectangle box(originBox , sceneSize.first, sceneSize.second, sceneSize.third, sceneResolution*2);

//            Object obj(Object::BIRD, Object::Pose::UNSPECIFIED, false, false, box);
//            scenes[positiveListFiles.size() + i] = Scene( originScene, box.depth(), box.height(), box.width(),
//                               negativeListFiles[i], {obj});
//        }


//        Mixture mixture( models);

//        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, nbIterations/nbIterations,
//                      nbDatamine, maxNegSample, C, J);

//        //TODO include initializeParts in train()
//        mixture.initializeParts( nbParts, chairPartSize);


//        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval,
//                      nbIterations, nbDatamine, maxNegSample, C, J);

    }

    void train( string dataFolder){

        int nbParts = 1;
        double C = 0.002, J = 2;
        float boxOverlap = 0.5;
        int interval = 1, nbIterations = 1, nbDatamine = 3, maxNegSample = 2000;
        int nbComponents = 1; //nb of object poses without symetry


        string xmlExtension = ".xml", pcExtension = ".ply";
        vector<string> sceneList;
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (dataFolder.c_str())) != NULL) {
          while ((ent = readdir (dir)) != NULL) {
              string fname = ent->d_name;
              if( fname.compare(".") != 0 && fname.compare("..") != 0 && ent->d_type == DT_DIR){
                    sceneList.push_back( fname);
        //            printf ("%s\n", (folder + ent->d_name).c_str());
              }
          }
          closedir (dir);
        } else {
          perror ("could not open directory");
        }

        vector<Scene> scenes( sceneList.size());

        for( int i=0; i < sceneList.size(); ++i){
            printf ("%s\n", sceneList[i].c_str());
            string filePath = dataFolder + sceneList[i] + "/" + sceneList[i];
            scenes[i] = Scene( filePath + xmlExtension, filePath + pcExtension, sceneResolution);
        }

        //////OLD
//        Model::triple<int, int, int> chairSize(8,10,6);//8,10,6 in lvl 1
//        Model::triple<int, int, int> chairPartSize(8,10,6);//8,10,6 in lvl 0
//        Model model( chairSize, 0);
//        std::vector<Model> models = { model};
//        Mixture mixture( models);
        //////
        Mixture mixture( nbComponents, scenes, Object::CHAIR, interval);


        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval, nbIterations/nbIterations,
                      nbDatamine, 2000, C, J, boxOverlap);

        cout<<"test::trained Root"<<endl;

        //TODO include initializeParts in train()
        mixture.initializeParts( nbParts/*, chairPartSize*/);

        cout<<"test::initializeParts"<<endl;

        mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval,
                      nbIterations, nbDatamine, maxNegSample, C, J, boxOverlap);

        cout<<"test::train finished"<<endl;

    }


    void detect(const Mixture & mixture, /*int depth, int height, int width, */int interval, const GSHOTPyramid & pyramid,
                double threshold, double overlap,/* const string image, */ostream & out,
                const string & images, vector<Detection> & detections, const Scene scene,
                Object::Name name = Object::CHAIR)
    {
        // Compute the scores
        vector<Tensor3DF> scores;
        vector<Mixture::Indices> argmaxes;
        vector<vector<vector<Model::Positions> > > positions;
        int nb = 6;

        mixture.computeScores( pyramid, scores, argmaxes, &positions);

        // Cache the size of the models
        vector<Model::triple<int, int, int> > sizes(mixture.models().size());

        for (int i = 0; i < sizes.size(); ++i)
            sizes[i] = mixture.models()[i].rootSize();


        // For each scale
        for (int lvl = 0; lvl < scores.size(); ++lvl) {
            const double scale = 1 / pow(2.0, static_cast<double>(lvl) / interval);
            int offz = floor(pyramid.sceneOffset_(0)*scale);
            int offy = floor(pyramid.sceneOffset_(1)*scale);
            int offx = floor(pyramid.sceneOffset_(2)*scale);

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

//                        PointType p = PointType();
//                        p.z = pyramid.keypoints_[1]->at(x + y * cols + z * rows * cols).z;
//                        p.y = pyramid.keypoints_[1]->at(x + y * cols + z * rows * cols).y;
//                        p.x = pyramid.keypoints_[1]->at(x + y * cols + z * rows * cols).x;
//                        p.r = score / maxi*255;
//                        p.g = score / maxi*255;
//                        p.b = score / maxi*255;
//                        scoreCloud->at(x + y * cols + z * rows * cols) = p;

                        cout<<"test:: scores = "<<score<<endl;

                        if (score >= threshold) {

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
        sort(detections.begin(), detections.end()/*, ascendingOrder*/);

        for (int i = 1; i < detections.size(); ++i){
            detections.resize(remove_if(detections.begin() + i, detections.end(),
                                        Intersector(detections[i - 1], overlap, true)) - detections.begin());
        }

        cout<<"test:: detections.size after intersection = "<<detections.size()<<endl;

        // Draw the detections
        /*if (detections.size() > nb)*/ {
            nb = std::min((int)detections.size(), nb);

            for (int i = 0; i < nb/*detections.size()*/; ++i) {

                const int x = detections[i].x;
                const int y = detections[i].y;
                const int z = detections[i].z;
                const int lvl = detections[i].lvl;

                const int argmax = argmaxes[lvl]()(z, y, x);

//                cout<<"test:: argmax = "<<argmax<<endl;
                cout<<"test:: detection score = "<<detections[i].score<<endl;


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


                    Eigen::Vector3i origin((zp+pyramid.sceneOffset_(0)/* + detections[i].origin()(0)*2*/)/*- pad.z()*/,
                                           (yp+pyramid.sceneOffset_(1) /*+ detections[i].origin()(1)*2*/)/*- pad.y()*/,
                                           (xp+pyramid.sceneOffset_(2) /*+ detections[i].origin()(2)*2*/)/* - pad.x()*/);
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
                    Vector3i(255,0,0));
        }
    }


    void test( string sceneName, string modelName){


        ifstream in( modelName);

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



        int interval = 1;
        float threshold=0/*.455*/, overlap=0.5;
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
//          transform.rotate (Eigen::AngleAxisf (1.57, Eigen::Vector3f::UnitX()));//bigScene
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
        Rectangle sceneRect(originScene, sceneSize.first, sceneSize.second, sceneSize.third, sceneResolution);

        ofstream out("tmpTest.txt");
        vector<Detection> detections;
        Scene scene( /*originScene, /*sceneSize.first, sceneSize.second, sceneSize.third,*/ sceneName, {});

        detect(mixture, interval, pyramid, threshold, overlap, /*file, */out,
               sceneName, detections, scene, Object::CHAIR);

        PointCloudPtr subspace(new PointCloudT());
        pcl::UniformSampling<PointType> sampling;
        sampling.setInputCloud(sceneCloud);
        sampling.setRadiusSearch (sceneResolution);
        sampling.filter(*subspace);

        cout<<"test:: model bias = "<<mixture.models()[0].bias()<<endl;

        viewer.addPC( subspace, 3);

        viewer.displayCubeLine(sceneRect);
//        Rectangle rootBox(Vector3i(mixture.models()[0].parts()[0].offset(0),
//                mixture.models()[0].parts()[0].offset(1),
//                mixture.models()[0].parts()[0].offset(2)-20),
//                      mixture.models()[0].rootSize().first, mixture.models()[0].rootSize().second,
//                      mixture.models()[0].rootSize().third, pyramid.resolutions()[1]);
//        viewer.displayCubeLine(rootBox,
//                               Eigen::Vector3f(pyramid.resolutions()[0],pyramid.resolutions()[0],pyramid.resolutions()[0]),
//                Vector3i(255,255,0));
//        for(int i=1;i<mixture.models()[0].parts().size();++i){
//            Rectangle partBox(Vector3i(mixture.models()[0].parts()[i].offset(0), mixture.models()[0].parts()[i].offset(1),
//                    mixture.models()[0].parts()[i].offset(2)-40),
//                          mixture.models()[0].partSize().first, mixture.models()[0].partSize().second,
//                          mixture.models()[0].partSize().third, pyramid.resolutions()[0]);
//            viewer.displayCubeLine(partBox,
//                                   Eigen::Vector3f(0,0,0),
//                    Vector3i(100,0,0));
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

    void checkSHOT(string pcName){

        int lumi = 5;
        int rows = 150, cols = GSHOTPyramid::DescriptorSize;
        int colsOff = 1;
        int nbKeyPts = 2;
        int nbKeyPtsX = nbKeyPts, nbKeyPtsY = nbKeyPts, nbKeyPtsZ = nbKeyPts;
        Mat img( rows, cols*nbKeyPtsX*nbKeyPtsY*nbKeyPtsZ, CV_8UC3, cv::Scalar(100,0,0));
        float rf2[9] = {1,0,0,0,1,0,0,0,1};
        RFType normLRF = RFType();
        normLRF.rf[0] = 0.05;
        normLRF.rf[1] = -0.72;
        normLRF.rf[2] = -0.68;
        normLRF.rf[3] = 0;
        normLRF.rf[4] = -1;
        normLRF.rf[5] = 0;
        normLRF.rf[6] = 0.03;
        normLRF.rf[7] = 0.68;
        normLRF.rf[8] = -0.73;

        float maxi = 0;

        PointCloudPtr cloud( new PointCloudT);
        if (readPointCloud(pcName, cloud) == -1) {
            cout<<"test::couldnt open pc file"<<endl;
        }

        PointType min;
        PointType max;
        pcl::getMinMax3D(*cloud, min, max);

        float descr_rad = std::max( (max.x - min.x) / (nbKeyPtsX + 1), std::max(
                                        (max.y - min.y) / (nbKeyPtsY + 1), (max.z - min.z) / (nbKeyPtsZ + 1)));

        cout<<"descr_rad : "<<descr_rad<<endl;

        PointCloudPtr keypoints (new PointCloudT (nbKeyPtsX*nbKeyPtsY*nbKeyPtsZ,1,PointType()));
        PointCloudRFPtr lrf (new PointCloudRF (nbKeyPtsX*nbKeyPtsY*nbKeyPtsZ,1,RFType()));

        for(int z=0;z<nbKeyPtsZ;++z){
            for(int y=0;y<nbKeyPtsY;++y){
                for(int x=0;x<nbKeyPtsX;++x){
                    PointType p = PointType();
                    p.x = min.x + (x+1) * (max.x - min.x) / (nbKeyPtsX + 1);
                    p.y = min.y + (y+1) * (max.y - min.y) / (nbKeyPtsY + 1);
                    p.z = min.z + (z+1) * (max.z - min.z) / (nbKeyPtsZ + 1);
                    keypoints->at(x + y * nbKeyPtsX + z * nbKeyPtsY * nbKeyPtsX) = p;
                    lrf->at(x + y * nbKeyPtsX + z * nbKeyPtsY * nbKeyPtsX) = normLRF;
                }
            }
        }

        //    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        //    transform.rotate (Eigen::AngleAxisf (1.57, Eigen::Vector3f::UnitY()));
        //    PointCloudPtr rotatedCloud (new PointCloudT);
        //    pcl::transformPointCloud (*cloud, *rotatedCloud, transform);


        DescriptorsPtr descriptors (new Descriptors());
        SurfaceNormalsPtr normals (new SurfaceNormals());

        pcl::NormalEstimationOMP<PointType,NormalType> norm_est;
        norm_est.setKSearch (8);
        norm_est.setInputCloud (cloud);
        norm_est.compute (*normals);

        pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
        descr_est.setRadiusSearch (descr_rad);
        descr_est.setInputCloud (keypoints);
        descr_est.setInputNormals (normals);
        descr_est.setSearchSurface (cloud);
//        descr_est.setInputReferenceFrames(lrf);
        descr_est.compute (*descriptors);

/////Display sum descriptor
//        for(int y=0;y<rows;++y){
//            for(int x=0;x<cols;++x){
//                Vec3b color = img.at<Vec3b>(Point(x,y));
//                float sumColor = 0;
//                for(int i=0;i<descriptors->size();++i){
//                    if (pcl_isnan(descriptors->points[i].descriptor[x])){
//                        descriptors->points[i].descriptor[x] = 0;
//                    }
//                    if( descriptors->points[i].descriptor[x] > maxi) maxi = descriptors->points[i].descriptor[x];
//                    sumColor += descriptors->points[i].descriptor[x];
//                }
//                color[0] = lumi*sumColor*255/descriptors->size();
//                color[1] = color[0];
//                color[2] = color[0];
//                for(int j=0;j<colsOff;++j){
//                    img.at<Vec3b>(Point(x*colsOff+j,y)) = color;
//                }
//            }
//        }
//        for(int y=rows-6;y<rows;++y){
//            for(int x=0;x<cols;++x){
//                Vec3b color = img.at<Vec3b>(Point(x,y));
//                if(x%32==0){
//                    color[2] = 255;//r
//                    color[1] = 0;//g
//                    color[0] = 0;//b
//                }else {
//                    color[2] = 0;
//                    if(x%2==0){
//                        color[1] = 255;
//                        color[0] = 0;
//                    }else {
//                        color[1] = 0;
//                        if(x%2==1)color[0] = 255;
//                        else color[0] = 0;
//                    }
//                }
//                for(int j=0;j<colsOff;++j){
//                    img.at<Vec3b>(Point(x*colsOff+j,y)) = color;
//                }
//            }
//        }
/////

///Display concatenate descriptor
        for(int y=0;y<rows;++y){
            for(int x=0;x<cols;++x){
                for(int i=0;i<descriptors->size();++i){
                    Vec3b color = img.at<Vec3b>(Point(x+i*cols,y));
                    if (pcl_isnan(descriptors->points[i].descriptor[x])){
                        descriptors->points[i].descriptor[x] = 0;
                    }
                    if( descriptors->points[i].descriptor[x] > maxi) maxi = descriptors->points[i].descriptor[x];
                    float desc = descriptors->points[i].descriptor[x];
                    color[0] = lumi*desc*desc*255;
                    color[1] = color[0];
                    color[2] = color[0];
                    img.at<Vec3b>(Point(x+i*cols,y)) = color;
                }
            }
        }

        for(int y=rows-6;y<rows;++y){
            for(int x=0;x<cols;++x){
                for(int i=0;i<descriptors->size();++i){
                    Vec3b color = img.at<Vec3b>(Point(x,y));
                    if(i%2==0){
                        color[2] = 255;//r
                        color[1] = 0;//g
                        color[0] = 0;//b
                    }else {
                        color[0] = 0;//r
                        color[1] = 255;//g
                        color[2] = 0;//b
                    }
                    img.at<Vec3b>(Point(x+i*cols,y)) = color;
                }
            }
        }
///
        float rf[9] = {0,0,0,0,0,0,0,0,0};
        cout<<"mean rf : ";
        for(int j=0;j<9;++j){
            int cpt = 0;
            for(int i=0;i<descriptors->size();++i){
                if (!pcl_isnan(descriptors->points[i].rf[j])){
                    rf[j] += descriptors->points[i].rf[j];
                    ++cpt;
                }
            }
            if(cpt) rf[j] /= cpt;
            cout<<rf[j]<<" ";
        }
        cout<<endl;

        cout<<"maxi descriptor value = "<<maxi<<endl;

        for(int i=0;i<descriptors->size();++i){
            viewer.displayAxis(descriptors->points[i].rf, pcl::PointXYZ(keypoints->points[i].x,
                                                                   keypoints->points[i].y,
                                                                   keypoints->points[i].z), descriptors->size());
        }
        viewer.displayAxis(rf);

        viewer.addPC( cloud);
        viewer.addPC( keypoints, 3, Eigen::Vector3i(255, 255, 255));
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


    void checkSHOTs(vector<string> pcNames){

        int lumi = 5;
        int rows = 150, cols = GSHOTPyramid::DescriptorSize;
        int nbKeyPts = 2;
        int nbKeyPtsX = nbKeyPts, nbKeyPtsY = nbKeyPts, nbKeyPtsZ = nbKeyPts;
        float rf[pcNames.size()*9];
        for(int p=0; p<pcNames.size();++p){
            for(int i=0; i<9;++i){
                rf[i+9*p] = 0;
            }
        }

        for(int p=0; p<pcNames.size();++p){
            Mat img( rows, cols*nbKeyPtsX*nbKeyPtsY*nbKeyPtsZ, CV_8UC3, cv::Scalar(100,0,0));

            float maxi = 0;


            PointCloudPtr cloud( new PointCloudT);
            if (readPointCloud(pcNames[p], cloud) == -1) {
                cout<<"test::couldnt open pc file"<<endl;
            }

            PointType min;
            PointType max;
            pcl::getMinMax3D(*cloud, min, max);

            float descr_rad = std::max( (max.x - min.x) / (nbKeyPtsX + 1), std::max(
                                            (max.y - min.y) / (nbKeyPtsY + 1), (max.z - min.z) / (nbKeyPtsZ + 1)));

            cout<<"descr_rad : "<<descr_rad<<endl;

            PointCloudPtr keypoints (new PointCloudT (nbKeyPtsX*nbKeyPtsY*nbKeyPtsZ,1,PointType()));

            for(int z=0;z<nbKeyPtsZ;++z){
                for(int y=0;y<nbKeyPtsY;++y){
                    for(int x=0;x<nbKeyPtsX;++x){
                        PointType p = PointType();
                        p.x = min.x + (x+1) * (max.x - min.x) / (nbKeyPtsX + 1);
                        p.y = min.y + (y+1) * (max.y - min.y) / (nbKeyPtsY + 1);
                        p.z = min.z + (z+1) * (max.z - min.z) / (nbKeyPtsZ + 1);
                        keypoints->at(x + y * nbKeyPtsX + z * nbKeyPtsY * nbKeyPtsX) = p;
                    }
                }
            }

            DescriptorsPtr descriptors (new Descriptors());
            SurfaceNormalsPtr normals (new SurfaceNormals());

            pcl::NormalEstimationOMP<PointType,NormalType> norm_est;
            norm_est.setKSearch (8);
            norm_est.setInputCloud (cloud);
            norm_est.compute (*normals);

            pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
            descr_est.setRadiusSearch (descr_rad);
            descr_est.setInputCloud (keypoints);
            descr_est.setInputNormals (normals);
            descr_est.setSearchSurface (cloud);
            descr_est.compute (*descriptors);

    ///Display concatenate descriptor
            for(int y=0;y<rows;++y){
                for(int x=0;x<cols;++x){
                    for(int i=0;i<descriptors->size();++i){
                        Vec3b color = img.at<Vec3b>(Point(x+i*cols,y));
                        if (pcl_isnan(descriptors->points[i].descriptor[x])){
                            descriptors->points[i].descriptor[x] = 0;
                        }
                        if( descriptors->points[i].descriptor[x] > maxi) maxi = descriptors->points[i].descriptor[x];
                        float desc = descriptors->points[i].descriptor[x];
                        color[0] = lumi*desc*desc*255;
                        color[1] = color[0];
                        color[2] = color[0];
                        img.at<Vec3b>(Point(x+i*cols,y)) = color;
                    }
                }
            }

            for(int y=rows-6;y<rows;++y){
                for(int x=0;x<cols;++x){
                    for(int i=0;i<descriptors->size();++i){
                        Vec3b color = img.at<Vec3b>(Point(x,y));
                        if(i%2==0){
                            color[2] = 255;//r
                            color[1] = 0;//g
                            color[0] = 0;//b
                        }else {
                            color[0] = 0;//r
                            color[1] = 255;//g
                            color[2] = 0;//b
                        }
                        img.at<Vec3b>(Point(x+i*cols,y)) = color;
                    }
                }
            }
    ///

            cout<<"mean rf : ";
            for(int j=0;j<9;++j){
                int cpt = 0;
                for(int i=0;i<descriptors->size();++i){
                    if (!pcl_isnan(descriptors->points[i].rf[j])){
                        rf[j+9*p] += descriptors->points[i].rf[j];
                        ++cpt;
                    }
                }
                if(cpt) rf[j+9*p] /= cpt;
                cout<<rf[j+9*p]<<" ";
            }
            cout<<endl;

            cout<<"maxi descriptor value = "<<maxi<<endl;

//            for(int i=0;i<descriptors->size();++i){
//                viewer.displayAxis(descriptors->points[i].rf, pcl::PointXYZ(keypoints->points[i].x,
//                                                                       keypoints->points[i].y,
//                                                                       keypoints->points[i].z), descriptors->size());
//            }
            viewer.displayAxis(rf+9*p);


            viewer.addPC( cloud);
            viewer.addPC( keypoints, 3, Eigen::Vector3i(255, 255, 255));

            if (img.empty()){
                cout << "\n Image not created. You"
                             " have done something wrong. \n";
                return;    // Unsuccessful.
            }

            string name = "img";
            imwrite( name.append(to_string(p)).append(".jpg"), img );
        }


        if(pcNames.size()>1){
            Matrix3f r0,r1;
            for(int i=0; i<3;++i){
                float aux0 = rf[i*3] + rf[1+i*3] + rf[2+i*3];
                float aux1 = rf[9+i*3] + rf[9+1+i*3] + rf[9+2+i*3];

                for(int j=0; j<3;++j){
                    r0(i,j) = rf[j+i*3]/*/aux0*/;
                    r1(j,i) = rf[9+j+i*3]/*/aux1*/;
                }
            }
            cout<<endl;


            Matrix3f rotation = r0*r1;
//            rotation <<0,-1,0,
//                    1,0,0,
//                    0,0,1;
            cout<<"r0 : "<<r0<<endl;
            cout<<"r1 : "<<r1<<endl;
            cout<<"rotation : "<<rotation<<endl;

            Vector3f ea = rotation.eulerAngles(0, 1, 2);
            cout<<"angles : ";
            for(int i=0; i<3;++i){
                float angle = ea[i]*180/M_PI;
                cout<<angle<<" ";
            }
            cout<<endl;

        }
    }

    float sceneResolution;
    Viewer viewer;
};


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


    Test test;

//    PointCloudPtr cloud( new PointCloudT);
//    if (readPointCloud("/home/ubuntu/3DDataset/3DDPM/chair/faceChair.pcd", cloud) == -1) {
//        cout<<"test::couldnt open pcd file"<<endl;
//    }
//    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//    transform.rotate (Eigen::AngleAxisf (-1.57, Eigen::Vector3f::UnitX()));//2rotChair
//    transform.rotate (Eigen::AngleAxisf (-1.57, Eigen::Vector3f::UnitY()));
////    transform.rotate (Eigen::AngleAxisf (-0.72, Eigen::Vector3f::UnitX()));//randomChair
////    transform.rotate (Eigen::AngleAxisf (3, Eigen::Vector3f::UnitY()));
////    transform.rotate (Eigen::AngleAxisf (-2.24, Eigen::Vector3f::UnitZ()));

//    PointCloudPtr rotatedCloud (new PointCloudT);
//    pcl::transformPointCloud (*cloud, *rotatedCloud, transform);
//    pcl::io::savePCDFileASCII ("/home/ubuntu/3DDataset/3DDPM/chair/2rotChair.pcd", *rotatedCloud);

    // testSceneMiddle_compress
    // smallScene2
    int start = getMilliCount();


//    test.oldTrain("/home/ubuntu/3DDataset/3DDPM/chair_normalized/",
//               "/media/ubuntu/DATA/3DDataset/ModelNet10_normalized/table/full/");
////               "/media/ubuntu/DATA/3DDataset/Cat51_normalized/monster_truck/full/");


//    test.train("/media/ubuntu/DATA/3DDataset/sceneNN+/");

//    test.test( "/media/ubuntu/DATA/3DDataset/sceneNN+/005/005.ply",
//               "tmp.txt");

    //smallSceneNN+/chair_part1_it1_weight15.txt

//    test.checkSHOT("/home/ubuntu/3DDataset/3DDPM/chair/faceChair.pcd");
//    test.checkSHOT("/home/ubuntu/3DDataset/3DDPM/chair/sideChair.pcd");
//    test.checkSHOT("/home/ubuntu/3DDataset/3DDPM/chair/upSideDownChair.pcd");
//    test.checkSHOT("/home/ubuntu/3DDataset/3DDPM/chair/2rotChair.pcd");

    test.checkSHOTs({"/home/ubuntu/3DDataset/3DDPM/chair/faceChair.pcd",
                     "/home/ubuntu/3DDataset/3DDPM/chair/sideChair.pcd"});


//    test.checkImages("/home/ubuntu/3DDataset/3DDPM/table/");

    // testSceneMiddle.ply // in color
    // testSceneMiddle_compress.pcd
    // smallScene4.pcd
    // scene_2.ply


//    PointCloudPtr cloud( new PointCloudT);
//    if (readPointCloud("/media/ubuntu/DATA/3DDataset/scenenn+/036/036.ply", cloud) == -1) {
//        cout<<"test::couldnt open pcd file"<<endl;
//    }

//    PointCloudPtr subspace(new PointCloudT());
//    pcl::UniformSampling<PointType> sampling;
//    sampling.setInputCloud(cloud);
//    sampling.setRadiusSearch (test.sceneResolution);
//    sampling.filter(*subspace);

//    test.viewer.addPC( subspace, 3);

//    Eigen::Vector3f boxCenter(1.13802, 0.156418, -1.69328);//0.382699, -0.00194225, 0.415045);//0.415045, -0.00194225, 0.382699);
//    Eigen::Vector3f boxSize(1.79048, 0.95146, -1.13563);//1.06383, 1.05044, 1.15705);//1.15705, 1.05044, 1.06383);
//    Eigen::Vector3f pose(0/*.996802*/, 0, -0/*.0799147*/);//1.06383, 1.05044, 1.15705);//1.15705, 1.05044, 1.06383);
//    Eigen::Matrix4f tform;
//    tform.setIdentity ();
//    tform.topLeftCorner (3, 3) = Eigen::Matrix3f (Eigen::Quaternionf (0,0,0,0));

//    Eigen::Vector3i origin( floor( ( boxCenter(2) ) / test.sceneResolution),
//                            floor( ( boxCenter(1)) / test.sceneResolution),
//                            floor( ( boxCenter(0)) / test.sceneResolution));
//    // absolute bndbox positions
//    Rectangle bndbox( origin, -( boxCenter(2)-boxSize(2)) / test.sceneResolution,
//                      -( boxCenter(1)-boxSize(1)+pose(1)) / test.sceneResolution,
//            -( boxCenter(0)-boxSize(0)+pose(0)) / test.sceneResolution,
//                      test.sceneResolution);
//    cout<<"bndbox : "<<bndbox<<endl;
//    test.viewer.displayCubeLine(bndbox);

//    PointCloudPtr cloud2( new PointCloudT (2,1,PointType()));
//    PointType p = PointType();
//    p.x = boxCenter(0);
//    p.y = boxCenter(1);
//    p.z = boxCenter(2);
//    cloud2->at(0) = p;
//    p.x = boxSize(0)+pose(0);
//    p.y = boxSize(1)+pose(1);
//    p.z = boxSize(2)+pose(2);
//    cloud2->at(1) = p;

//    PointCloudPtr orientedCloud (new PointCloudT);
//    pcl::transformPointCloud (*cloud2, *orientedCloud, tform);

//    test.viewer.addPC( cloud2, 8, Eigen::Vector3i(255, 255, 0));
//    test.viewer.addPC( orientedCloud, 8, Eigen::Vector3i(0, 255, 255));




    int end = getMilliCount();

    cout << "Time : " << end-start << endl;

//    ifstream in("tableModel4parts1pos.txt");

//    if (!in.is_open()) {
//        cerr << "Cannot open model file\n" << endl;
//    }

//    Mixture mixTable;
//    in >> mixTable;

//    if (mixTable.empty()) {
//        cerr << "Invalid model file\n" << endl;
//    }

//    ifstream in2("chairModel4parts1pos.txt");

//    if (!in2.is_open()) {
//        cerr << "Cannot open model file\n" << endl;
//    }

//    Mixture mixChair;
//    in2 >> mixChair;

//    if (mixChair.empty()) {
//        cerr << "Invalid model file\n" << endl;
//    }

//    cout<<"Chair dot Table = " << mixChair.models()[0].dot(mixTable.models()[0]) << endl;
//    cout<<"Table dot Chair = " << mixTable.models()[0].dot(mixChair.models()[0]) << endl;
//    cout<<"Table dot Table = " << mixTable.models()[0].dot(mixTable.models()[0]) << endl;
//    cout<<"Chair dot Chair = " << mixChair.models()[0].dot(mixChair.models()[0]) << endl;

//    cout<<"Chair norm = " << mixChair.models()[0].norm() << endl;
//    cout<<"Table norm = " << mixTable.models()[0].norm() << endl;

//    cout<<"Part 0 agglo : " << endl;

//    cout<<"Chair dot Table = " << mixChair.models()[0].parts()[0].filter.agglomerate().dot(mixTable.models()[0].parts()[0].filter.agglomerate()) << endl;
//    cout<<"Table dot Chair = " << mixTable.models()[0].parts()[0].filter.agglomerate().dot(mixChair.models()[0].parts()[0].filter.agglomerate()) << endl;
//    cout<<"Table dot Table = " << mixTable.models()[0].parts()[0].filter.agglomerate().dot(mixTable.models()[0].parts()[0].filter.agglomerate()) << endl;
//    cout<<"Chair dot Chair = " << mixChair.models()[0].parts()[0].filter.agglomerate().dot(mixChair.models()[0].parts()[0].filter.agglomerate()) << endl;


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


