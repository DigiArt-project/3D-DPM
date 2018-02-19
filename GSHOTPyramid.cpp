
#include "GSHOTPyramid.hpp"

using namespace Eigen;
using namespace FFLD;
using namespace std;


GSHOTPyramid::GSHOTPyramid(): _octaves(0), _original_resolution(0),height_(0)
{
    _descriptors = new std::vector<typename pcl::PointCloud<DescriptorType>::Ptr >();
    _keypoints = new std::vector<typename pcl::PointCloud<PointType>::Ptr >();
}

GSHOTPyramid::GSHOTPyramid(const GSHOTPyramid &fp)
{
    *this = fp;
}

GSHOTPyramid::~GSHOTPyramid()
{
    _descriptors->clear();
    _keypoints->clear();
    height_ = 0;
}

GSHOTPyramid::GSHOTPyramid(typename pcl::PointCloud<PointType>::Ptr input, int octaves, int interval, float starting_resolution, float starting_kp_grid_reso, float starting_descr_rad): _octaves(octaves), _original_resolution(starting_resolution)
{
    interval_ = interval;
    if(octaves < 0 || interval < 0 || starting_resolution < 0) {
        std::cerr << "Arguments of the constructor cannot be negative!" << std::endl;
        exit(0);
    }
    if(octaves ==0) {
        std::cerr << "Not very useful to build a pyramid with no octaves..." << std::endl;
        exit(0);
    }
    
    _descriptors = new std::vector<typename pcl::PointCloud<DescriptorType>::Ptr >();
    _keypoints = new std::vector<typename pcl::PointCloud<PointType>::Ptr >();
    
    PointType min;
    PointType max;
    
    typename pcl::PointCloud<PointType>::Ptr normalized_input(new pcl::PointCloud<PointType>());
    
    //Define a resolution standard. Not advised to use full resolution.
    if(starting_resolution>0) {
        
        pcl::UniformSampling<PointType> pre_sampling;
        pre_sampling.setInputCloud (input);
        pre_sampling.setRadiusSearch (starting_resolution);
        pre_sampling.filter (*normalized_input);
        
        pcl::getMinMax3D(*normalized_input,min,max);
        
        _keypoints->push_back(this->compute_keypoints(normalized_input, starting_kp_grid_reso, min, max));
        _descriptors->push_back(this->compute_space(normalized_input,_keypoints->at(0),starting_descr_rad));
    }
    else {
        typename pcl::PointCloud<PointType>::Ptr normalized_input = input;
        
        pcl::getMinMax3D(*normalized_input,min,max);
        
        _keypoints->push_back(this->compute_keypoints(normalized_input, starting_kp_grid_reso, min, max));
        _descriptors->push_back(this->compute_space(normalized_input,_keypoints->at(0),starting_descr_rad));
    }
    float resolution;
    float kp_resolution;
    float descr_rad;
    
    for(unsigned int i=1;i<=octaves;i++){
        
        resolution = starting_resolution*pow(2,i-1);
        kp_resolution = starting_kp_grid_reso*i;
        descr_rad = starting_descr_rad*i;
        
        for(unsigned int j=0;j<=interval;j++){
            
            typename pcl::PointCloud<PointType>::Ptr subspace(new typename pcl::PointCloud<PointType>());
            float subresolution = resolution;
            float sub_kp_resolution = kp_resolution;
            float sub_descr_rad = descr_rad;
            pcl::UniformSampling<PointType> sampling;
            
            subresolution += (j+1)*resolution/(interval+1);
            sub_kp_resolution += (j+1)*kp_resolution/(interval+1);
            sub_descr_rad += (j+1)*descr_rad/(interval+1);
            
            sampling.setInputCloud(normalized_input);
            sampling.setRadiusSearch (subresolution);
            sampling.filter(*subspace);
            
            _keypoints->push_back(compute_keypoints(normalized_input, sub_kp_resolution, min, max));
            _descriptors->push_back(compute_space(subspace,_keypoints->back(),sub_descr_rad));
        }
    }
    height_ = _descriptors->size();
}

/*
 * WARNING : need to build a sub structure to partially specialize the pyramid
 */
typename pcl::PointCloud<DescriptorType>::Ptr GSHOTPyramid::compute_space(typename pcl::PointCloud<PointType>::Ptr input, typename pcl::PointCloud<PointType>::Ptr keypoints, float descr_rad)
{
    pcl::PointCloud<DescriptorType>::Ptr descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<NormalType>::Ptr normals (new pcl::PointCloud<NormalType> ());

    //pcl::SHOTEstimation<PointType, NormalType, DescriptorType> norm_est;
    pcl::NormalEstimation<PointType,NormalType> norm_est;
    norm_est.setKSearch (8);
    norm_est.setInputCloud (input);
    norm_est.compute (*normals);

    pcl::SHOTEstimation<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad);
    descr_est.setInputCloud (keypoints);
    descr_est.setInputNormals (normals);
    descr_est.setSearchSurface (input);
    descr_est.compute (*descriptors);
    
    return descriptors;
}

typename pcl::PointCloud<PointType>::Ptr GSHOTPyramid::compute_keypoints(typename pcl::PointCloud<PointType>::Ptr input, float grid_reso, PointType min, PointType max) {
    
    int pt_nb_x = (int)((max.x-min.x)/grid_reso+1);
    int pt_nb_y = (int)((max.y-min.y)/grid_reso+1);
    int pt_nb_z = (int)((max.z-min.z)/grid_reso+1);
    int pt_nb = pt_nb_x*pt_nb_y*pt_nb_z;
    typename pcl::PointCloud<PointType>::Ptr keypoints (new pcl::PointCloud<PointType> (pt_nb,1,PointType()));
    
    unsigned int i;
#if defined(_OPENMP)
#pragma omp parallel for num_threads(omp_get_max_threads())
#endif
    for(i=0;i<pt_nb_x;i++){
        unsigned int j;
        for(j=0;j<pt_nb_y;j++){
            unsigned int k;
            for(k=0;k<pt_nb_z;k++){
                PointType p = PointType();
                p.x = min.x + i*grid_reso;
                p.y = min.y + j*grid_reso;
                p.z = min.z + k*grid_reso;
                keypoints->at(pt_nb_y*pt_nb_z*i + pt_nb_z*j + k) = p;
            }
        }
    }
    
    return keypoints;
}

pcl::PointCloud<DescriptorType> GSHOTPyramid::get_descriptors_layer(unsigned int i)
{
    return *(_descriptors->at(i));
}

pcl::PointCloud<PointType> GSHOTPyramid::get_keypoints_layer(unsigned int i)
{
    return *(_keypoints->at(i));
}

int GSHOTPyramid::get_octaves()
{
    return _octaves;
}
int GSHOTPyramid::get_sub_between_octaves()
{
    return (_descriptors->size()-1)/(_octaves+1);
}

int GSHOTPyramid::get_height()
{
    return height_;
}

float GSHOTPyramid::get_original_resolution()
{
    return _original_resolution;
}

float GSHOTPyramid::get_layer_resolution(int i)
{
    int oct = (int) (i/get_octaves());
    int pos_in_octave = i%get_octaves();
    float dist_btw_octave = _original_resolution/pow(2,oct) - _original_resolution/pow(2,oct+1);
    return _original_resolution/pow(2,oct) - dist_btw_octave*pos_in_octave/(get_sub_between_octaves()+1);
}
/*
 * Need to declare every descriptor we intend to use
 */
const char* GSHOTPyramid::get_descriptor_type()
{
    //return typeid(DescriptorType).name();
    return "SHOT352";
}

const char* GSHOTPyramid::get_point_type()
{
    //return typeid(PointType).name();
    return "XRZRGB";
}

int GSHOTPyramid::interval() const
{
    return interval_;
}

bool GSHOTPyramid::empty() const
{
    //return levels().empty();
}

void GSHOTPyramid::toString()
{
    
    std::cout<<"Pyramid specifications :"<<std::endl<<std::endl;
    std::cout<<"Descriptor type : "<<this->get_descriptor_type()<<std::endl;
    std::cout<<"Point type : "<<this->get_point_type()<<std::endl;
    std::cout<<"Number of octaves : "<<this->_octaves<<std::endl;
    std::cout<<"Height : "<<this->_descriptors->size()<<" layers ("<<get_sub_between_octaves()<<" subsections between octaves)"<<std::endl;
    if(_original_resolution==0) {
        std::cout<<"No presampling used, pyramid starting at the original resolution."<<std::endl;
    }
    else {
        std::cout << "Resolution of the first floor : "<<_original_resolution<<std::endl;
    }
}





