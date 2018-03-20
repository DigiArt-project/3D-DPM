#include "Model.h"

#include <pcl/io/pcd_io.h>

#include <cstdlib>
#include <sys/timeb.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;

    class Test{
    public:
        Test(){
        }


    private:
        std::vector<Model::Part> parts_;
        double bias_;
    };

int main(){

    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);




    Eigen::Vector3i pad(1,1,1);
    Model::triple<int,int,int> rootSize(5,5,5);
    Model::triple<int,int,int>partSize( 10,10,10);
    Model model( rootSize);


    Model sample;
    PointCloudPtr cloud(new PointCloudT);
//    pcl::io::loadPCDFile( "/home/ubuntu/3DDataset/3DDPM/scene.pcd", *cloud);
    pcl::io::loadPCDFile( "/home/ubuntu/3DDataset/3DDPM/chair.pcd", *cloud);

    int lvl = 0, x = 0, y = 0, z= 0;
    GSHOTPyramid scenePyramid(cloud, pad);
    model.initializeSample(scenePyramid, x, y, z, lvl,sample);

    cout<< scenePyramid.levels()[1].size() <<endl;

    GSHOTPyramid::Level root2x = scenePyramid.levels()[1];


     model.initializeParts( 2, partSize, root2x);

//    cout<< cloud->size() <<endl;
    vector<Tensor3DF> scores(5);

    //TODO
    sample.convolve(scenePyramid, scores);

    cout<< "score : "<<scores[0].size()<<endl;
    
    return 0;
}

