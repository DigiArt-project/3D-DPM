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
    int box;
    Detection() : Rectangle(), score(0), x(0), y(0), z(0), lvl(0), box(0)
    {
    }

    Detection(Rectangle bndbox, GSHOTPyramid::Scalar score, int z, int y, int x, int lvl, int box) : Rectangle(bndbox),
    score(score), x(x), y(y), z(z), lvl(lvl), box(box)
    {
    }

    bool operator<(const Detection & detection) const
    {
        return detection.score < score && !( score < detection.score);
    }
};
struct AscendingOrder{
    bool operator()( const struct Detection score1, const struct Detection score2) const{
        return score2.score < score1.score && !( score1.score < score2.score);
    }
};


class Test{
public:

    Test()
    {

        sceneResolution = 0.2;//0.09/2.0;
        cout<<"test::sceneResolution : "<<sceneResolution<<endl;

    }

    void oldTrain(string positiveFolder, string negativeFolder){

//        int nbParts = 3;
//        double C = 0.002, J = 2;
//        int interval = 1, nbIterations = 3, nbDatamine = 3, maxNegSample = 2000;
//        triple<int, int, int> chairSize(8,10,6);//8,10,6 in lvl 1
//        triple<int, int, int> chairPartSize(8,10,6);//8,10,6 in lvl 0
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
//            triple<int, int, int> sceneSize = chairSize;//( resolution(0)/2.0, resolution(1)/2.0, resolution(2)/2.0);

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
//            triple<int, int, int> sceneSize = chairSize;//( resolution(0)/2.0, resolution(1)/2.0, resolution(2)/2.0);

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

        int nbParts = 2;
        double C = 0.002, J = 2;
        float boxOverlap = 0.25;
        int interval = 1, nbIterations = 2, nbDatamine = 3, maxNegSample = 2000;
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
//        triple<int, int, int> chairSize(8,10,6);//8,10,6 in lvl 1
//        triple<int, int, int> chairPartSize(8,10,6);//8,10,6 in lvl 0
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


    void detect(const Mixture & mixture, int interval, const GSHOTPyramid & pyramid,
                double threshold, double overlap, ostream & out,
                const string & images, vector<Detection> & detections2, const Scene scene,
                Object::Name name = Object::CHAIR)
    {
        // Compute the scores
        vector<Detection> detections;

        vector<vector<Tensor3DF> >scores;
        vector<Mixture::Indices> argmaxes;
        vector<vector<vector<vector<Model::Positions> > > >positions;
        int nb = 6;

        mixture.computeScores( pyramid, scores, argmaxes, &positions);

        // Cache the size of the models
        vector<triple<int, int, int> > sizes(mixture.models().size());

        for (int i = 0; i < sizes.size(); ++i)
            sizes[i] = mixture.models()[i].rootSize();


        // For each scale
        for (int lvl = 0; lvl < scores.size(); ++lvl) {
            const double scale = 1 / pow(2.0, static_cast<double>(lvl) / interval);

            for (int box = 0; box < scores[lvl].size(); ++box) {


//                const PointCloudConstPtr boxCloud = pyramid.keypoints_[lvl][box];
//                PointType min;
//                PointType max;
//                pcl::getMinMax3D(*boxCloud, min, max);

                const int depths = scores[lvl][box].depths();
                const int rows = scores[lvl][box].rows();
                const int cols = scores[lvl][box].cols();

                cout<<"test:: for lvl "<<lvl<<" :"<<endl;

//                cout<<"test:: scores[lvl].depths() = "<<depths<<endl;
//                cout<<"test:: scores[lvl].rows() = "<<rows<<endl;
//                cout<<"test:: scores[lvl].cols() = "<<cols<<endl;

                if(scores[lvl][box].size() > 0){
                    ofstream out("conv.txt");
                    out << scores[lvl][box]();


    //                float maxi = scores[lvl][box].max();

    //                PointCloudPtr scoreCloud(new PointCloudT());
    //                scoreCloud->width    = scores[lvl].size();
    //                scoreCloud->height   = 1;
    //                scoreCloud->points.resize (scoreCloud->width);


                    const double score = scores[lvl][box]()(0,0,0);

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

                        Rectangle bndbox = pyramid.rectangles_[lvl][box];

                        cout<<"test:: detection bndbox = "<<bndbox<<endl;

                        if (!bndbox.empty()){
                            detections.push_back(Detection(pyramid.rectangles_[lvl][box], score, 0, 0, 0, lvl, box));
                            cout<<"test:: bndbox added to detections"<<endl;
                        }

                    }

        //            viewer.addPC(scoreCloud, 4, Eigen::Vector3i(255, 255, 255));

                }
            }
        }

        cout<<"test:: detections.size = "<<detections.size()<<endl;
        // Non maxima suppression
        std::sort(detections.begin(), detections.end(), AscendingOrder());
//        for(int i=0; i<detections.size();++i){
//            cout<<"score Detect : "<<detections[i].rec.cloud()->size()<<endl;
//        }
        cout<<"test::sort done"<<endl;

//        vector<Detection>::iterator it;
//        for (it = detections.begin()+1; it != detections.end(); /*++it*/){
//            cout<<"Detect size : "<<detections.size()<<endl;
//            Intersector inter(*(it-1), overlap, true);
//            if( inter(*it)){
//                cout<<"erase"<<endl;
//                it = detections.erase(it);
//                cout<<"erase done"<<endl;
//            } else{
//                ++it;
//            }
//        }

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
                const int box = detections[i].box;

                const int argmax = argmaxes[lvl]()(z, y, x);

//                cout<<"test:: argmax = "<<argmax<<endl;
                cout<<"test:: detection score = "<<detections[i].score<<" for box : "<<detections[i].box<<endl;


                //draw each parts
                for (int j = 0; j < positions[argmax].size(); ++j) {

                    const int zp = positions[argmax][j][lvl][box]()(z, y, x)(0);
                    const int yp = positions[argmax][j][lvl][box]()(z, y, x)(1);
                    const int xp = positions[argmax][j][lvl][box]()(z, y, x)(2);
                    const int lvlp = positions[argmax][j][lvl][box]()(z, y, x)(3);

//                    cout<<"test: part position : "<<zp<<" "<<yp<<" "<<xp<<" "<<lvlp<<endl;

                    int pt_nb_y = pyramid.topology_[lvlp](1);
                    int pt_nb_x = pyramid.topology_[lvlp](2);
                    PointType p = pyramid.keypoints_[lvlp][box]->points[xp + yp * pt_nb_x + zp * pt_nb_y * pt_nb_x];
                    Eigen::Vector3f origin(0,0,0);
                    Eigen::Vector3f boxSize(
                                mixture.models()[argmax].partSize().third * pyramid.resolutions()[lvlp],
                                mixture.models()[argmax].partSize().second * pyramid.resolutions()[lvlp],
                                mixture.models()[argmax].partSize().first * pyramid.resolutions()[lvlp]);

                    /////////
                    Matrix4f trans = pyramid.rectangles_[lvl][box].transform();
                    trans.topRightCorner(3, 1) += Vector3f(xp * pyramid.resolutions()[lvlp],
                                                           yp * pyramid.resolutions()[lvlp],
                                                           zp * pyramid.resolutions()[lvlp]);
                    /////////

                    Rectangle bndbox( origin, boxSize, trans);//indices of the cube in the PC


//                    cout<<"test:: part bndbox to draw = "<<bndbox<<endl;
            //TODO add rotation to drawing
                    viewer.displayCubeLine(bndbox, Vector3i(100,0,255));
                }

                // Draw the root last
//                cout<<"test:: root bndbox = "<<detections[i]<<endl;


                Eigen::Vector3f origin(0,0,0);
                Eigen::Vector3f boxSize(
                            sizes[argmaxes[lvl]()(z, y, x)].third * pyramid.resolutions()[lvl],
                            sizes[argmaxes[lvl]()(z, y, x)].second * pyramid.resolutions()[lvl],
                            sizes[argmaxes[lvl]()(z, y, x)].first * pyramid.resolutions()[lvl]);
                Rectangle bbox( origin, boxSize, pyramid.rectangles_[lvl][box].transform());
                if( i < 2){
                    viewer.displayCubeLine(bbox, Vector3i(255,0,0));
                }else{
                    viewer.displayCubeLine(bbox, Vector3i(0,255,255));
                }

            }

        }
//        if( detections.size() > 0){
//            cout<<"test:: root bndbox = "<<detections[0]<<" with score : "<<detections[0].score<<endl;
//            Rectangle box(Vector3i(detections[0].origin()(0), detections[0].origin()(1), detections[0].origin()(2)),
//                          mixture.models()[0].rootSize().first,
//                          mixture.models()[0].rootSize().second,
//                          mixture.models()[0].rootSize().third, pyramid.resolutions()[1]);
//            viewer.displayCubeLine(box,
//                                   Eigen::Vector3f(pyramid.resolutions()[0],pyramid.resolutions()[0],pyramid.resolutions()[0]),
//                    Vector3i(255,0,0));
//        }
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
        float threshold=0, overlap=0.1;
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

        int maxFilterSizes = max( mixture.models()[0].rootSize().first, max( mixture.models()[0].rootSize().second,
                mixture.models()[0].rootSize().third));
        triple<int, int, int> filterSizes(maxFilterSizes,maxFilterSizes,maxFilterSizes);

        const GSHOTPyramid pyramid(sceneCloud, mixture.models()[0].rootSize(), interval, sceneResolution);

        PointType minScene;
        PointType maxScene;
        pcl::getMinMax3D(*sceneCloud , minScene, maxScene);

        Vector3f originScene( minScene.z,
                               minScene.y,
                               minScene.x);
        Vector3f sceneSize(maxScene.z-minScene.z,
                           maxScene.y-minScene.y,
                           maxScene.x-minScene.x);
        Rectangle sceneRect(originScene, sceneSize);
        cout<<"sceneRect : "<<sceneRect<<endl;

        PointCloudPtr test (new PointCloudT(1,1,PointType()));
        int boxNb = 700;//5+3*12+5*12*7;//700;449
        test->points[0] = pyramid.globalKeyPts->points[boxNb];
        viewer.addPC( test, 7, Eigen::Vector3i(255, 0, 0));
        viewer.addPC( pyramid.keypoints_[1][boxNb], 5, Eigen::Vector3i(255, 255, 0));
//        Rectangle rect = Rectangle();
//        rect.setCloud(pyramid.keypoints_[1][boxNb]);
//        viewer.displayCubeLine(pyramid.rectangles_[1][boxNb]);
        cout<<"lrf : ";
        for(int i=0;i<9;++i){
            cout<<pyramid.globalDescriptors->points[boxNb].rf[i]<<" ";
        }
        cout<<endl;
        viewer.addPC( pyramid.globalKeyPts, 2, Eigen::Vector3i(255, 255, 255));
        viewer.displayAxis(pyramid.globalDescriptors->points[boxNb].rf);
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

//        viewer.displayCubeLine(sceneRect);
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
//        vector<string> positiveListFiles;
//        DIR *dir;
//        struct dirent *ent;
//        if ((dir = opendir (positiveFolder.c_str())) != NULL) {
//          while ((ent = readdir (dir)) != NULL) {
//              if( string(ent->d_name).compare(".") != 0 && string(ent->d_name).compare("..") != 0)
//                positiveListFiles.push_back( positiveFolder + ent->d_name);
////            printf ("%s\n", (folder + ent->d_name).c_str());
//          }
//          closedir (dir);
//        } else {
//          perror ("could not open directory");
//        }

//        ifstream in("tmp.txt");

//        if (!in.is_open()) {
//            cerr << "Cannot open model file\n" << endl;
//            return;
//        }

//        Mixture mixture;
//        in >> mixture;

//        if (mixture.empty()) {
//            cerr << "Invalid model file\n" << endl;
//            return;
//        }


//        int nbParts = 4 + 1;
//        Mat img( positiveListFiles.size() + 1, nbParts * GSHOTPyramid::DescriptorSize, CV_8UC3, cv::Scalar(0,0,0));

//        int interval = 1;

//        for(int y=0;y<positiveListFiles.size();y++){
//            float maxi = 0;

//            PointCloudPtr cloud( new PointCloudT);
//            if (readPointCloud(positiveListFiles[y], cloud) == -1) {
//                cout<<"test::couldnt open pcd file"<<endl;
//            }

//            int maxFilterSizes = max( mixture.models()[0].rootSize().first, max( mixture.models()[0].rootSize().second,
//                    mixture.models()[0].rootSize().third));
//            triple<int, int, int> filterSizes(maxFilterSizes,maxFilterSizes,maxFilterSizes);

//            const GSHOTPyramid pyramid(cloud, filterSizes, interval);

//            GSHOTPyramid::Level lvl = pyramid.levels()[1].agglomerate();


//            for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
//                if( maxi < lvl()(0,0,0)(x)) maxi = lvl()(0,0,0)(x);
//            }
//            cout<<"maxi descriptor value = "<<maxi<<endl;


//            for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
//                Vec3b color = img.at<Vec3b>(Point(x,y));
//                color[0] = lvl()(0,0,0)(x)/maxi*255/*pyramid.levels()[1].size()*/;
//                color[1] = color[0];
//                color[2] = color[0];
//                img.at<Vec3b>(Point(x,y)) = color;
//            }

//            for(int p=1;p<nbParts;p++){
//                maxi = 0;
//                GSHOTPyramid::Level lvl = pyramid.levels()[0].agglomerateBlock(
//                        mixture.models()[0].parts()[p].offset(0),
//                        mixture.models()[0].parts()[p].offset(1),
//                        mixture.models()[0].parts()[p].offset(2),
//                        mixture.models()[0].partSize().first,
//                        mixture.models()[0].partSize().second,
//                        mixture.models()[0].partSize().third);

//                for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
//                    if( maxi < lvl()(0,0,0)(x)) maxi = lvl()(0,0,0)(x);
//                }
//                cout<<"maxi descriptor value = "<<maxi<<endl;

//                for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
//                    Vec3b color = img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,y));
//                    color[0] = lvl()(0,0,0)(x)/maxi*255/*/mixture.models()[0].parts()[p].filter.size()*/;
//                    color[1] = color[0];
//                    color[2] = color[0];
//                    img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,y)) = color;
//                }

//            }

//        }


//        for(int p=0;p<nbParts;p++){

//            GSHOTPyramid::Level mod = mixture.models()[0].parts()[p].filter.agglomerate();

//            float maxi = 0, mini = mod()(0,0,0)(0);
//            for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
//                if( maxi < mod()(0,0,0)(x)) maxi = mod()(0,0,0)(x);
//                if( mini > mod()(0,0,0)(x)) mini = mod()(0,0,0)(x);
//            }
//            cout<<"maxi model descriptor value = "<<maxi<<endl;
//            cout<<"mini model descriptor value = "<<mini<<endl;

//            for(int x=0;x<GSHOTPyramid::DescriptorSize;x++){
//                if(mod()(0,0,0)(x) > 0){
//                    Vec3b color = img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,positiveListFiles.size()));
//                    color[2] = mod()(0,0,0)(x)/maxi*255;//*mixture.models()[0].parts()[p].filter.size()*/;
//                    img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,positiveListFiles.size())) = color;
//                } else{
//                    Vec3b color = img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,positiveListFiles.size()));
//                    color[1] = mod()(0,0,0)(x)/mini*255;//*mixture.models()[0].parts()[p].filter.size()*/;
//                    img.at<Vec3b>(Point(x+p*GSHOTPyramid::DescriptorSize,positiveListFiles.size())) = color;
//                }
//            }

//        }

//        if (img.empty()){
//            cout << "\n Image not created. You"
//                         " have done something wrong. \n";
//            return;    // Unsuccessful.
//        }

//        imwrite( "img.jpg", img );

//        namedWindow("A_good_name", CV_WINDOW_AUTOSIZE);

//        imshow("A_good_name", img);
////        resizeWindow("A_good_name", 600,600);

//        waitKey(0); //wait infinite time for a keypress

//        destroyWindow("A_good_name");
    }

    void checkSHOT(string pcName){

        int lumi = 5;
        int rows = 150, cols = GSHOTPyramid::DescriptorSize;
        int colsOff = 1;
        int nbKeyPts = 1;
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
        int nbKeyPts = 1;
        int nbKeyPtsX = nbKeyPts, nbKeyPtsY = nbKeyPts, nbKeyPtsZ = nbKeyPts;
        float rf[pcNames.size()*9];
        for(int p=0; p<pcNames.size();++p){
            for(int i=0; i<9;++i){
                rf[i+9*p] = 0;
            }
        }

        PointCloudPtr keypoints (new PointCloudT (nbKeyPtsX*nbKeyPtsY*nbKeyPtsZ,1,PointType()));

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

            cout<<"min : "<<min<<endl;


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
//            viewer.displayAxis(rf+9*p);


//            viewer.addPC( cloud);
//            viewer.addPC( keypoints, 3, Eigen::Vector3i(255, 255, 255));

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
                for(int j=0; j<3;++j){
                    r0(j,i) = rf[j+i*3];
                    r1(j,i) = rf[9+j+i*3];
                }
            }
            cout<<endl;


            Matrix4f res;
            res.setIdentity ();
            res.topLeftCorner (3, 3) = r0;
            Vector3f origin( 0,0,0);
            Vector3f trans( 1,0,1);

            Matrix4f tf = GSHOTPyramid::getNormalizeTransform(rf, &rf[9], origin, trans);//from 1 to 2

            cout<<"r1 : "<<endl<<r1<<endl;
            cout<<"r0 : "<<endl<<tf*res<<endl;
//            cout<<"rotation : "<<endl<<tf<<endl;

            PointCloudPtr cloud1( new PointCloudT);
            if (readPointCloud(pcNames[0], cloud1) == -1) {
                cout<<"test::couldnt open pcd file"<<endl;
            }
            PointCloudPtr cloud2( new PointCloudT);
            if (readPointCloud(pcNames[1], cloud2) == -1) {
                cout<<"test::couldnt open pcd file"<<endl;
            }

            PointCloudPtr rotatedCloud (new PointCloudT);
            pcl::transformPointCloud (*cloud1, *rotatedCloud, tf);
            viewer.addPC( cloud1);
            viewer.addPC( cloud2, 3, Eigen::Vector3i(0,0,255));
            viewer.addPC( rotatedCloud, 3, Eigen::Vector3i(255, 0, 0));

        }
    }

    void checkIntersector(){

        Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();
        Matrix3f rotation1;
        rotation1 = AngleAxisf(0, Vector3f::UnitX())
                  * AngleAxisf(1.57/2.0,  Vector3f::UnitY())
                  * AngleAxisf(0, Vector3f::UnitZ());
        transform1.topLeftCorner (3, 3) = rotation1;
        Vector3f origin1(0,0,0);
        Vector3f recSize1(2,2,2);
        Rectangle rec1( origin1, recSize1, transform1);

        Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();
        Matrix3f rotation2;
        rotation2 = AngleAxisf(1.57/2.0, Vector3f::UnitX())
                  * AngleAxisf(0,  Vector3f::UnitY())
                  * AngleAxisf(0, Vector3f::UnitZ());
        transform2.topLeftCorner (3, 3) = rotation2;
        Vector3f origin2(0,-0.2,1);
        Vector3f recSize2(2,2,2);
        Rectangle rec2( origin2, recSize2, transform2);

        Intersector inter(rec1, 0.4);

        double score = 0;
        if( inter(rec2, &score)){
            cout<<"Intersection true"<<endl;
        }else{
            cout<<"Intersection false"<<endl;
        }
        cout<<"Score : "<<score<<endl;

        viewer.displayCubeLine(rec1);
        viewer.displayCubeLine(rec2);
        viewer.addPC(inter.intersectionCloud_);
    }

    float sceneResolution;
    Viewer viewer;
};


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


    Test test;

//    PointCloudPtr cloud( new PointCloudT);
//    if (readPointCloud("/home/ubuntu/3DDataset/3DDPM/chair/2rotChair.pcd", cloud) == -1) {
//        cout<<"test::couldnt open pcd file"<<endl;
//    }
//    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//    transform.rotate (Eigen::AngleAxisf (-1.57, Eigen::Vector3f::UnitX()));//2rotChair
//    transform.rotate (Eigen::AngleAxisf (-1.57, Eigen::Vector3f::UnitY()));
////    transform.rotate (Eigen::AngleAxisf (-0.72, Eigen::Vector3f::UnitX()));//randomChair
////    transform.rotate (Eigen::AngleAxisf (3, Eigen::Vector3f::UnitY()));
////    transform.rotate (Eigen::AngleAxisf (-2.24, Eigen::Vector3f::UnitZ()));

//    Eigen::Matrix4f tform;
//    tform.setIdentity();
////    tform.topLeftCorner (3, 3) = transform;
//    Vector3f translation( 1,0,1);
//    tform.topRightCorner(3, 1) = translation ;
//    PointCloudPtr rotatedCloud (new PointCloudT);
//    pcl::transformPointCloud (*cloud, *rotatedCloud, tform);
//    pcl::io::savePCDFileASCII ("/home/ubuntu/3DDataset/3DDPM/chair/2rotChairTranslated.pcd", *rotatedCloud);

    // testSceneMiddle_compress
    // smallScene2
    int start = getMilliCount();


//    test.oldTrain("/home/ubuntu/3DDataset/3DDPM/chair_normalized/",
//               "/media/ubuntu/DATA/3DDataset/ModelNet10_normalized/table/full/");
////               "/media/ubuntu/DATA/3DDataset/Cat51_normalized/monster_truck/full/");


    ///train and test not working if run in serie because of PointCloudPtr in Rectangle
    /// change it to PointCloudT if you want to solve it
    test.train("/media/ubuntu/DATA/3DDataset/smallSceneNN+/");

    test.test( "/media/ubuntu/DATA/3DDataset/sceneNN+/005/005.ply",
               "tmp.txt");


//    Vector3f origin(0,0,0);
//    Vector3f recSize(5,5,5);
//    Rectangle rec( origin, recSize);
//    vector<Detection> detections;
//    for(int i = 0; i < 1000; ++i){
//        detections.push_back( Detection(rec, 5, 0,0,0,1,1));
//    }

//    std::sort(detections.begin(), detections.end());
//    vector<Detection>::iterator it;
//    for (it = detections.begin()+1; it != detections.end(); /*++it*/){
//        cout<<"Detect size : "<<detections.size()<<endl;
//        Intersector inter(*(it-1), 0.5, true);
//        if( inter(*it)){
//            cout<<"erase"<<endl;
//            it = detections.erase(it);
//            cout<<"erase done"<<endl;
//        } else{
//            ++it;
//        }
//    }

//    test.checkIntersector();//to improve

//    Eigen::Vector3f origin1( 0.415045, -0.00194225, 0.382699);
//    // absolute bndbox positions
//    Rectangle bndbox1( origin1, 0.742005, 1.05238, 0.681131,
//                      test.sceneResolution);
//    Eigen::Vector3f origin2( -1.16383, -0.0100944, 0.935017);
//    // absolute bndbox positions
//    Rectangle bndbox2( origin2, 0.743034, 1.06102, 0.719493,
//                      test.sceneResolution);
//    test.viewer.displayCubeLine(bndbox1);
//    test.viewer.displayCubeLine(bndbox2);



//    test.checkSHOT("/home/ubuntu/3DDataset/3DDPM/chair/faceChair.pcd");
//    test.checkSHOT("/home/ubuntu/3DDataset/3DDPM/chair/sideChair.pcd");
//    test.checkSHOT("/home/ubuntu/3DDataset/3DDPM/chair/upSideDownChair.pcd");
//    test.checkSHOT("/home/ubuntu/3DDataset/3DDPM/chair/2rotChair.pcd");

//    test.checkSHOTs({"/home/ubuntu/3DDataset/3DDPM/chair/faceChair.pcd",
//                     "/home/ubuntu/3DDataset/3DDPM/chair/2rotChairTranslated.pcd"});


//    test.checkImages("/home/ubuntu/3DDataset/3DDPM/table/");

    // testSceneMiddle.ply // in color
    // testSceneMiddle_compress.pcd
    // smallScene4.pcd
    // scene_2.ply


//    PointCloudPtr cloud( new PointCloudT);
//    if (readPointCloud("/media/ubuntu/DATA/3DDataset/sceneNN+/005/005.ply", cloud) == -1) {
//        cout<<"test::couldnt open pcd file"<<endl;
//    }

//    PointCloudPtr subspace(new PointCloudT());
//    pcl::UniformSampling<PointType> sampling;
//    sampling.setInputCloud(cloud);
//    sampling.setRadiusSearch (test.sceneResolution);
//    sampling.filter(*subspace);

//    test.viewer.addPC( subspace, 3);

//    Eigen::Vector3f boxCenter(0.382699, -0.00194225, 0.415045);
//    Eigen::Vector3f boxSize(1.06383, 1.05044, 1.15705);
//    Eigen::Vector3f size(boxSize(0) - boxCenter(0), boxSize(1) - boxCenter(1), boxSize(2) - boxCenter(2));

////    Eigen::Vector3f pose(0/*.996802*/, 0, -0/*.0799147*/);//1.06383, 1.05044, 1.15705);//1.15705, 1.05044, 1.06383);
////    Eigen::Matrix4f tform;
////    tform.setIdentity ();
////    tform.topLeftCorner (3, 3) = Eigen::Quaternionf (0.231075, 0.972684, -0.00420024, -0.0217461 ).toRotationMatrix();

//    PointCloudPtr keyPt( new PointCloudT(1, 1, PointType()));
//    PointType p = PointType();
//    p.x = boxCenter(0) + (boxSize(0) - boxCenter(0))/2.0;
//    p.y = boxCenter(1) + (boxSize(1) - boxCenter(1))/2.0;
//    p.z = boxCenter(2) + (boxSize(2) - boxCenter(2))/2.0;

//    keyPt->points[0] = p;
//    DescriptorsPtr descriptors (new Descriptors());
//    SurfaceNormalsPtr normals (new SurfaceNormals());

//    pcl::NormalEstimationOMP<PointType,NormalType> norm_est;
//    norm_est.setKSearch (8);
//    norm_est.setInputCloud (cloud);
//    norm_est.compute (*normals);

//    float descr_rad = std::min(size(0), std::min(size(1),size(2)));

//    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
//    descr_est.setRadiusSearch (descr_rad);
//    descr_est.setInputCloud (keyPt);
//    descr_est.setInputNormals (normals);
//    descr_est.setSearchSurface (cloud);
//    descr_est.compute (*descriptors);

//    float rf[9];
//    cout<<"mean rf : ";
//    for(int j=0;j<9;++j){
//        for(int i=0;i<descriptors->size();++i){
//            if (!pcl_isnan(descriptors->points[i].rf[j])){
//                rf[j] = descriptors->points[i].rf[j];
//            }
//        }
//        cout<<rf[j]<<" ";
//    }
//    cout<<endl;

//    Eigen::Vector3i origin( floor( ( boxCenter(2) ) / test.sceneResolution),
//                            floor( ( boxCenter(1)) / test.sceneResolution),
//                            floor( ( boxCenter(0)) / test.sceneResolution));
//    // absolute bndbox positions
//    Rectangle bndbox( origin, ( boxSize(2)) / test.sceneResolution,
//                      ( boxSize(1)) / test.sceneResolution,
//            ( boxSize(0)) / test.sceneResolution,
//                      test.sceneResolution);
//    cout<<"bndbox : "<<bndbox<<endl;
//    test.viewer.displayCubeLine(bndbox);

//    PointCloudPtr cloud2( new PointCloudT (8,1,PointType()));
//    PointType p = PointType();
//    p.x = boxCenter(0);
//    p.y = boxCenter(1);
//    p.z = boxCenter(2);
//    cloud2->at(0) = p;
//    p.x = boxCenter(0)+boxSize(0);
//    p.y = boxCenter(1)+boxSize(1);
//    p.z = boxCenter(2)+boxSize(2);
//    cloud2->at(1) = p;
//    p.x = boxCenter(0);
//    p.y = boxCenter(1)+boxSize(1);
//    p.z = boxCenter(2)+boxSize(2);
//    cloud2->at(2) = p;
//    p.x = boxCenter(0)+boxSize(0);
//    p.y = boxCenter(1);
//    p.z = boxCenter(2)+boxSize(2);
//    cloud2->at(3) = p;
//    p.x = boxCenter(0)+boxSize(0);
//    p.y = boxCenter(1)+boxSize(1);
//    p.z = boxCenter(2);
//    cloud2->at(4) = p;
//    p.x = boxCenter(0);
//    p.y = boxCenter(1);
//    p.z = boxCenter(2)+boxSize(2);
//    cloud2->at(5) = p;
//    p.x = boxCenter(0);
//    p.y = boxCenter(1)+boxSize(1);
//    p.z = boxCenter(2);
//    cloud2->at(6) = p;
//    p.x = boxCenter(0)+boxSize(0);
//    p.y = boxCenter(1);
//    p.z = boxCenter(2);
//    cloud2->at(7) = p;



//    PointCloudPtr orientedCloud (new PointCloudT);
//    pcl::transformPointCloud (*cloud2, *orientedCloud, tform);

//    test.viewer.addPC( cloud2, 8, Eigen::Vector3i(255, 255, 0));
//    test.viewer.addPC( orientedCloud, 8, Eigen::Vector3i(0, 255, 255));




    int end = getMilliCount();

    cout << "Time : " << end-start << endl;


//    PointCloudPtr cloud( new PointCloudT);
//    if (readPointCloud("/home/ubuntu/3DDataset/3DDPM/table.pcd", cloud) == -1) {
//        cout<<"test::couldnt open pcd file"<<endl;
//    }


    test.viewer.show();

    return 0;
}


