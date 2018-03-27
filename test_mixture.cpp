
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
///////////////////

//    float resolution = starting_resolution;
//    float grid_reso = resolution;
//    float descr_rad = resolution;
//    PointCloudPtr subspace(new PointCloudT());

//    pcl::UniformSampling<PointType> sampling;
//    PointType min;
//    PointType max;
//    sampling.setInputCloud(cloud);
//    sampling.setRadiusSearch (resolution);
//    sampling.filter(*subspace);

//    pcl::getMinMax3D(*subspace, min, max);
//    int pt_nb_x = (int)((max.x-min.x)/grid_reso+1);
//    int pt_nb_y = (int)((max.y-min.y)/grid_reso+1);
//    int pt_nb_z = (int)((max.z-min.z)/grid_reso+1);
//    int pt_nb = pt_nb_x*pt_nb_y*pt_nb_z;


//    PointCloudPtr keypoints (new PointCloudT (pt_nb,1,PointType()));

//    for(int i=0;i<pt_nb_z;++i){
//        for(int j=0;j<pt_nb_y;++j){
//            for(int k=0;k<pt_nb_x;++k){
//                PointType p = PointType();
//                p.x = min.x + k*grid_reso;
//                p.y = min.y + j*grid_reso;
//                p.z = min.z + i*grid_reso;
//                keypoints->at(k + j * pt_nb_x + i * pt_nb_y * pt_nb_x) = p;
//            }
//        }
//    }

//    DescriptorsPtr descriptors (new Descriptors());
//    SurfaceNormalsPtr normals (new SurfaceNormals());

//    //pcl::SHOTEstimation<PointType, NormalType, DescriptorType> norm_est;
//    pcl::NormalEstimation<PointType,NormalType> norm_est;
//    norm_est.setKSearch (8);
//    norm_est.setInputCloud (cloud);
//    norm_est.compute (*normals);

//    pcl::SHOTEstimation<PointType, NormalType, DescriptorType> descr_est;
//    descr_est.setRadiusSearch (descr_rad);
//    descr_est.setInputCloud (keypoints);
//    descr_est.setInputNormals (normals);
//    descr_est.setSearchSurface (cloud);
//    descr_est.compute (*descriptors);



//    int nb_kpt = keypoints->size();

//    bool isZero = true;
//    for (size_t i = 0; i < descriptors->size(); ++i){
//        for (size_t j = 0; j < DescriptorType::descriptorSize(); ++j){

//            if (pcl_isnan(descriptors->points[i].descriptor[j])){
//                descriptors->points[i].descriptor[j] = 0;
//            }

//            cout << "GSHOTPyr::constructor descriptors->points["<<i<<"].descriptor["<<j<<"] "
//                 << descriptors->points[i].descriptor[j] << endl;
//            if( descriptors->points[i].descriptor[j] != 0) isZero = false;
//        }

//    }

//    cout << " isZero : " << isZero << endl;



///////////////////////


    PointType min;
    PointType max;
    pcl::getMinMax3D(*cloud , min, max);
    float resolution = 10 * starting_resolution;
    Model::triple<int, int, int> sceneSize( (max.z-min.z)/resolution+1,
                                           (max.y-min.y)/resolution+1,
                                           (max.x-min.x)/resolution+1);
    Model::triple<int, int, int> rootSize( sceneSize.first/2,
                                           sceneSize.second/2,
                                           sceneSize.third/2);
    Model::triple<int, int, int> partSize( sceneSize.first/3,
                                           sceneSize.second/3,
                                           sceneSize.third/3);
    Rectangle sceneRec( Eigen::Vector3i(1, 1, 1), rootSize.third, rootSize.second, rootSize.first);

    cout<<"test::sceneRec : "<< sceneRec <<endl;
    cout<<"test::sceneSize : "<<sceneSize.first<<" "<<sceneSize.second<<" "<<sceneSize.third<<endl;
    cout<<"test::rootSize : "<<rootSize.first<<" "<<rootSize.second<<" "<<rootSize.third<<endl;
    cout<<"test::partSize : "<<partSize.first<<" "<<partSize.second<<" "<<partSize.third<<endl;

    Model model( rootSize, 1, partSize);
    std::vector<Model> models = { model,
                                  model};
    cout<<"test::filter size : "<<models[1].parts()[0].filter.size()<<endl;
    cout<<"test::model empty : "<<models.empty()<<endl;

    vector<Object> objects;
    Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, sceneRec);
    objects.push_back(obj);
    vector<Scene> scenes = {Scene( sceneSize.third, sceneSize.second, rootSize.first, filename, objects)};


    Mixture mixture( models);

    int interval = 1;
    mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval);




    return 0;
}

