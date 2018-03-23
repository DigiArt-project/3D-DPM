
#include "Mixture.h"

#include <cstdlib>
#include <sys/timeb.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;


int main(){
    //Turn pcl message to OFF !!!!!!!!!!!!!!!!!!!!
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    float starting_resolution = 0.07;


    char* filename = "/home/ubuntu/3DDataset/3DDPM/chair.pcd";
    PointCloudPtr cloud( new PointCloudT);
    if (pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1) {
        cout<<"test::couldnt open pcd file"<<endl;
        return 0;
    }


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
    Model::triple<int, int, int> rootSize( (max.z-min.z)/starting_resolution/1.5,
                                           (max.y-min.y)/starting_resolution/1.5,
                                           (max.x-min.x)/starting_resolution/1.5);
    Rectangle rec( min.x, max.y, min.z, rootSize.third, rootSize.second, rootSize.first);

    cout<<"test::rootSize : "<<rootSize.first<<" "<<rootSize.second<<" "<<rootSize.third<<endl;

    Model model( rootSize, 2, Model::triple<int, int, int>( rootSize.first/3, rootSize.second/3, rootSize.third/3));
    std::vector<Model> models = { model,
                                  model};
    cout<<"test::filter size : "<<models[1].parts()[0].filter.size()<<endl;
    cout<<"test::model empty : "<<models.empty()<<endl;

    vector<Object> objects;
    Object obj(Object::CHAIR, Object::Pose::UNSPECIFIED, false, false, rec);
    objects.push_back(obj);
    vector<Scene> scenes = {Scene( rootSize.third, rootSize.second, rootSize.first, filename, objects)};


    Mixture mixture( models);

    int interval = 2;
    mixture.train(scenes, Object::CHAIR, Eigen::Vector3i( 3,3,3), interval);




    return 0;
}

