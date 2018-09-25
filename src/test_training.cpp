// Written by Fisichella Thomas
// Date 25/05/2018

#include "Mixture.h"
#include "Intersector.h"
#include "Object.h"


#include <cstdlib>
#include <sys/timeb.h>

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

    Tensor3DF createScene(float scene[], int nx, int ny){
        Tensor3DF res(nx, ny, 1);
        float *data = res.tensor.data();
        for(int y=0; y < ny; ++y){
            for(int x=0; x < nx; ++x){
                data[x+y*nx] = scene[x+y*nx];
            }
        }
        return res;
    }

    Model createModel(float scene[], int nx, int ny){
        GSHOTPyramid::Level lvl(nx, ny, 1);
        GSHOTPyramid::Cell *data = lvl.tensor.data();
        for(int y=0; y < ny; ++y){
            for(int x=0; x < nx; ++x){
                data[x+y*nx](0) = scene[x+y*nx];
            }
        }

        Model::Part part;
        part.filter = lvl;
        part.offset(0) = 0;
        part.offset(1) = 0;
        part.offset(2) = 0;
        part.offset(3) = 0;
        part.deformation << -0.01, 0.0, -0.01, 0.0, -0.01, 0.0, -0.01, 0.0;

        return Model({part});
    }

    void trainMixture( Mixture& mix, vector<pair<Model, int> > positives, vector<pair<Model, int> > negatives,
                       int nbRelabel, int nbDatamine, int maxNegSample,
                       double C, double J){
        ///////

        double loss = numeric_limits<double>::infinity();

        for (int relabel = 0; relabel < nbRelabel; ++relabel) {
            cout<<"Mix::train relabel : "<< relabel <<endl;

            // Previous loss on the cache
            double prevLoss = -numeric_limits<double>::infinity();

            for (int datamine = 0; datamine < nbDatamine; ++datamine) {

                const int maxIterations =
                    min(max(10.0 * sqrt(static_cast<double>(positives.size())), 100.0), 1000.0);

                loss = mix.trainSVM(positives, negatives, C, J, maxIterations);

                cout << "Relabel: " << relabel << ", datamine: " << datamine
                     << ", # positives: " << positives.size() << ", # hard negatives: "
                     << (negatives.size()) << " (new) = "
                     << negatives.size() << ", loss (cache): " << loss << endl;


                // Save the latest model so as to be able to look at it while training
                ofstream out("tmp.txt");

                out << mix;

                // Stop if we are not making progress
                if ((0.999 * loss < prevLoss) && (negatives.size() < maxNegSample)){
                    cout<<"Mix::train stop because not making progress"<<endl;
                    break;
                }

                prevLoss = loss;
            }
        }

        //////
    }

    void train( vector<pair<Model, int> > positives, vector<pair<Model, int> > negatives){

        int nbParts = 0;
        double C = 0.002, J = 2;
        float boxOverlap = 0.5;
        int interval = 1, nbIterations = 1, nbDatamine = 1, maxNegSample = 2000;
        int nbComponents = 1; //nb of object poses without symetry

        Model model( triple<int,int,int>(3,3,1));
        Mixture mixture( {model});


        trainMixture(mixture, positives, negatives, nbIterations/nbIterations,
                      nbDatamine, maxNegSample, C, J);



        mixture.models()[0].initializeParts(nbParts, triple<int,int,int>(2,2,1));

        cout<<"test::initializeParts"<<endl;

        trainMixture(mixture, positives, negatives, nbIterations,
                      nbDatamine, maxNegSample, C, J);
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
        vector<triple<int, int, int> > sizes(mixture.models().size());

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
        triple<int, int, int> sceneSize( ceil((maxScene.z-originScene(0)*sceneResolution)/sceneResolution)+1,
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

    float sceneResolution;
    Viewer viewer;
};


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


    Test test;

    int start = getMilliCount();

    float dataScene1[7*3] = {0,0,1, 0,0,0,0,
                             0,1,0, 1,1,0,1,
                             1,0,0, 1,0,0,1};
    float dataPositive1[3*3] =  {0,0,1,
                                 0,1,0,
                                 1,0,0};
    float dataPositive2[3*3] =  {0,0,1,
                                 0,0,0,
                                 1,0,0};

    float dataNegative1[3*3] =  {0,0,0,
                                 1,1,1,
                                 0,0,0};
    float dataNegative2[3*3] =  {0,0,0,
                                 0,1,1,
                                 0,1,0};

    Tensor3DF scene1 = test.createScene( dataScene1, 7, 3);
    Model positive1 = test.createModel( dataPositive1, 3, 3);
    Model positive2 = test.createModel( dataPositive2, 3, 3);
    Model negative1 = test.createModel( dataNegative1, 3, 3);
    Model negative2 = test.createModel( dataNegative2, 3, 3);




    cout<<scene1.tensor<<endl;

    vector<Scene> scenes;
//    Object obj(Name name, Pose pose, bool truncated, bool difficult, Rectangle bndbox);
//    scenes.push_back(Scene("none", {obj}));


    vector<pair<Model, int> > positives;
    vector<pair<Model, int> > negatives;
    positives.push_back(pair<Model, int>(positive1, 0));
//    positives.push_back(pair<Model, int>(positive2, 0));
    negatives.push_back(pair<Model, int>(negative1, 0));
//    negatives.push_back(pair<Model, int>(negative2, 0));

    test.train( positives, negatives);

//    test.test( "/media/ubuntu/DATA/3DDataset/sceneNN+/005/005.ply",
//               "tmp.txt");

    int end = getMilliCount();

    cout << "Time : " << end-start << endl;

    test.viewer.show();

    return 0;
}


