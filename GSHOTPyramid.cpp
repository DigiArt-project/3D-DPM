
#include "GSHOTPyramid.h"

using namespace Eigen;
using namespace FFLD;
using namespace std;

///** CONSTRUCTORS**/
//#pragma mark -  CONSTRUCTORS METHODS

const int GSHOTPyramid::DescriptorSize;

GSHOTPyramid::GSHOTPyramid() : pad_( Eigen::Vector3i(0, 0, 0)), interval_(0)
{
}

GSHOTPyramid::GSHOTPyramid(const PointCloudPtr input, Eigen::Vector3i pad, int interval, float starting_resolution, int nbOctave):
pad_( Eigen::Vector3i(0, 0, 0)), interval_(0)
{
    if (input->empty() || (pad.x() < 1) || (pad.y() < 1) || (pad.z() < 1) || (interval < 1)) {
        cerr << "Attempting to create an empty pyramid" << endl;
        return;
    }
    
    pad_ = pad;
    interval_ = interval;
    
    
    //Exemple of starting resolution
//    float starting_kp_reso = 0.2;
//    float starting_descr_rad = 0.4;
    float resolution;
    starting_resolution = computeCloudResolution(input);//0.00338117;//
    cout << "GSHOTPyr::constructor starting_resolution : "<<starting_resolution<<endl;
//    float kp_resolution;
//    float descr_rad;


    levels_.resize( interval_ * nbOctave);
    resolutions_.resize( interval_ * nbOctave);
    keypoints_.resize( interval_ * nbOctave);
    topology.resize( interval_ * nbOctave);
    
    //TODO #pragma omp parallel for i
    for (int i = 0; i < interval_; ++i) {
        for (int j = 0; j < nbOctave; ++j) {
            int index = i + j * interval_;
//            resolution = 10 * starting_resolution * pow(2.0, -static_cast<double>(i) / interval) / pow(2.0, j);
            resolution = 25 * starting_resolution / pow(2.0, -static_cast<double>(i) / interval) * pow(2.0, j);



            PointCloudPtr subspace(new PointCloudT());
            pcl::UniformSampling<PointType> sampling;
            sampling.setInputCloud(input);
            sampling.setRadiusSearch (resolution);
            sampling.filter(*subspace);

            resolutions_[index] = resolution;
//            pcl::VoxelGrid<PointCloudT> sor;
//            sor.setInputCloud (input);
//            sor.setLeafSize (resolution, resolution, resolution);
//            sor.filter (*subspace);

            cout << "GSHOTPyr::constructor radius resolution at lvl "<<i<<" = "<<resolution<<endl;
            cout << "GSHOTPyr::constructor lvl size : "<<subspace->size()<<endl;
            cout << "GSHOTPyr::constructor index "<<index<<endl;

            PointType min;
            PointType max;
            pcl::getMinMax3D(*subspace, min, max);

            keypoints_[index] = compute_keypoints(subspace, resolution, min, max, index);
            DescriptorsPtr descriptors = compute_descriptor(subspace, keypoints_[index], 10*resolution);

            bool isZero = true;
            Level level( topology[index](0), topology[index](1), topology[index](2));
            Cell* levelCell = &(level()(0));
            //Loop over the number of keypoints available
            for (int kpt = 0; kpt < keypoints_[index]->size(); ++kpt){
                //For each keypoint, create a cell which will contain the corresponding shot descriptor
                Cell cell(GSHOTPyramid::DescriptorSize);
                //Then for each value of the associated descriptor, fill in the cell
                for( int k = 0; k < GSHOTPyramid::DescriptorSize; ++k){
                    cell.row(k) = descriptors->points[kpt].descriptor[k];
    //                cout << "GSHOTPyr::constructor descriptors->points["<<kpt<<"].descriptor["<<k<<"] "
    //                     << descriptors->points[kpt].descriptor[k] << endl;
    //                cout << "GSHOTPyr::constructor cell.row("<<k<<") "
    //                     << cell.row(k) << endl;
                    if( descriptors->points[kpt].descriptor[k] != 0) isZero = false;
                }
               //Add the cell to the current level
                //TODO check the order of the cells
                *levelCell = cell;
                ++levelCell;
            }
            //Once the first level is done, push it to the array of level
            levels_[index] = level;
            cout << "GSHOTPyr::constructor pyramid octave "<<j<<" isZero : " << isZero << endl;
        }
    }
}


void GSHOTPyramid::convolve(const Level & filter, vector<Tensor3DF >& convolutions) const
{

    convolutions.resize(levels_.size());

#pragma omp parallel for
    for (int i = 0; i < levels_.size(); ++i){
        cout<<"GSHOTPyramid::convolve filter.size() : "<< filter.size()
           << " with levels_[" <<i<< "].size() : " << levels_[i].size() << endl;
        Convolve(levels_[i], filter, convolutions[i]);
    }
}

//TODO urgent !!!!! remove the result of the convolution for x = [1 and 351]%352
//Its a correlation not a convolution
void GSHOTPyramid::Convolve(const Level & x, const Level & y, Tensor3DF & z)
{
    // Nothing to do if x is smaller than y
    if ((x().dimension(0) < y().dimension(0)) || (x().dimension(1) < y().dimension(1) ) || (x().dimension(2) < y().dimension(2) )) {
        cout<<"GSHOTPyramid::convolve error : level size is smaller than filter" << endl;
        cout<<"GSHOTPyramid::convolve error : " <<x().dimension(0)<<" < "<<y().dimension(0)
           <<" / " << x().dimension(1)<<" < "<<y().dimension(1)
          <<" / " << x().dimension(2)<<" < "<<y().dimension(2)<< endl;
        return;
    }

    Eigen::Tensor<float, 3, RowMajor> dx(x().dimension(0), x().dimension(1), x().dimension(2) * DescriptorSize),
                                      dy(y().dimension(0), y().dimension(1), y().dimension(2) * DescriptorSize);

//TODO use TensorMap
    bool xIsZero = true;
    for (int i = 0; i < x().dimension(0); ++i) {
        for (int j = 0; j < x().dimension(1); ++j) {
            for (int k = 0; k < x().dimension(2); ++k) {
                for (int l = 0; l < DescriptorSize; ++l) {
                    dx(i,j, k * DescriptorSize + l) = x()(i,j,k).coeff(l);
                    if( x()(i,j,k).coeff(l) != 0) xIsZero = false;
                }
            }
        }
    }
    cout<<"GSHOTPyramid::convolve level.isZero() : "<< xIsZero << endl;

    bool yIsZero = true;
    for (int i = 0; i < y().dimension(0); ++i) {
        for (int j = 0; j < y().dimension(1); ++j) {
            for (int k = 0; k < y().dimension(2); ++k) {
                for (int l = 0; l < DescriptorSize; ++l) {
                    dy(i,j, k * DescriptorSize + l) = y()(i,j,k).coeff(l);
                    if( y()(i,j,k).coeff(l) != 0) yIsZero = false;
                }
            }
        }
    }
    cout<<"GSHOTPyramid::convolve filter.isZero() : "<< yIsZero << endl;

    z().resize( dx.dimension(0) - dy.dimension(0) + 1,
              dx.dimension(1) - dy.dimension(1) + 1,
              dx.dimension(2) - dy.dimension(2) + 1);

    cout<<"GSHOTPyramid::convolve results.size() : "<< z.size() << endl;
    Eigen::array<ptrdiff_t, 3> dims({0, 1, 2});
    z() = dx.convolve(dy, dims);
}

//void GSHOTPyramid::Convolve(const Level & x, const Level & y, Tensor & z)
//{
//    // Nothing to do if x is smaller than y
//    if ((x.rows() < y.rows()) || (x.cols() < y.cols())) {
//        z = Tensor();
//        return;
//    }

//    z = Tensor::Zero( x.dimension(0) - y.dimension(0) + 1, x.dimension(1) - y.dimension(1) + 1, x.dimension(2) - y.dimension(2) + 1);

//    for (int i = 0; i < z.rows(); ++i) {
//        for (int j = 0; j < y.rows(); ++j) {
//            const Eigen::Map<const Matrix, Aligned, OuterStride<DescriptorSize> >
//                mapx(reinterpret_cast<const Scalar *>(x.row(i + j).data()), z.cols(), y.cols() * DescriptorSize);

//            const Eigen::Map<const RowVectorXf, Aligned>
//                mapy(reinterpret_cast<const Scalar *>(y.row(j).data()), y.cols() * DescriptorSize);

//            z.row(i).noalias() += mapy * mapx.transpose();
//        }
//    }
//}

//TODO: do we need the flip function ?
//GSHOTPyramid::Level GSHOTPyramid::Flip(const GSHOTPyramid::Level & level)
//{
//    //TODO: adapt --> 352 for shot
//    // Symmetric features
//    const int symmetry[GSHOTPyramid::DescriptorSize] =
//    {
//        9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 17, 16, 15, 14, 13, 12, 11, 10, // Contrast-sensitive
//        18, 26, 25, 24, 23, 22, 21, 20, 19, // Contrast-insensitive
//        28, 27, 30, 29, // Texture
//#ifndef FFLD_HOGPYRAMID_EXTRA_FEATURES
//        31 // Truncation
//#else
//        31, 32, 33, 34, 35, 36, 37, 38, 39, 40, // Uniform LBP
//        41, 42, 43, 44, 45, 46, // Color
//        47 // Truncation
//#endif
//    };

//    // Symmetric filter
//    GSHOTPyramid::Level result(level.rows(), level.cols(), level.depths());

//    for (int y = 0; y < level.rows(); ++y)
//        for (int x = 0; x < level.cols(); ++x)
//            for (int z = 0; z < level.depths(); ++z)
//                for (int i = 0; i < GSHOTPyramid::DescriptorSize; ++i)
//                    result(y, x, z)(i) = level(y, level.cols() - 1 - x,z)(symmetry[i]);

//    return result;
//}



//GSHOTPyramid::GSHOTPyramid(): _octaves(0), _original_resolution(0),height_(0),levels_(0,0,0)
//{
//    _descriptors = new std::vector<typename pcl::PointCloud<DescriptorType>::Ptr >();
//    _keypoints = new std::vector<typename pcl::PointCloud<PointType>::Ptr >();

//}

//GSHOTPyramid::GSHOTPyramid(const GSHOTPyramid &fp):levels_(0,0,0)
//{
//    *this = fp;
//}

//GSHOTPyramid::GSHOTPyramid(pcl::PointCloud<PointType>::Ptr input, int octaves, int interval, float starting_resolution, float starting_kp_grid_reso, float starting_descr_rad): _octaves(octaves), _original_resolution(starting_resolution),levels_(0,0,0)
//{
//    interval_ = interval;
    
//    if(octaves < 0 || interval < 0 || starting_resolution < 0) {
//        std::cerr << "Arguments of the constructor cannot be negative!" << std::endl;
//        exit(0);
//    }
//    if(octaves ==0) {
//        std::cerr << "Not very useful to build a pyramid with no octaves..." << std::endl;
//        exit(0);
//    }
    
//    _descriptors = new std::vector<typename pcl::PointCloud<DescriptorType>::Ptr >();
//    _keypoints = new std::vector<typename pcl::PointCloud<PointType>::Ptr >();
    
//    PointType min;
//    PointType max;
    
//    typename pcl::PointCloud<PointType>::Ptr normalized_input(new pcl::PointCloud<PointType>());
    
//    //Define a resolution standard. Not advised to use full resolution.
//    if(starting_resolution>0) {
        
//        pcl::UniformSampling<PointType> pre_sampling;
//        pre_sampling.setInputCloud (input);
//        pre_sampling.setRadiusSearch (starting_resolution);
//        pre_sampling.filter (*normalized_input);
        
//        pcl::getMinMax3D(*normalized_input,min,max);
        
//        _keypoints->push_back(this->compute_keypoints(normalized_input, starting_kp_grid_reso, min, max));
//        pcl::PointCloud<DescriptorType>::Ptr desc = this->compute_descriptor(normalized_input,_keypoints->at(0),starting_descr_rad);
//        _descriptors->push_back(desc);
//    }
//    else {
//        typename pcl::PointCloud<PointType>::Ptr normalized_input = input;
        
//        pcl::getMinMax3D(*normalized_input,min,max);
        
//        _keypoints->push_back(this->compute_keypoints(normalized_input, starting_kp_grid_reso, min, max));
//        pcl::PointCloud<DescriptorType>::Ptr desc = this->compute_descriptor(normalized_input,_keypoints->at(0),starting_descr_rad);
//        _descriptors->push_back(desc);
//    }
//    float resolution;
//    float kp_resolution;
//    float descr_rad;
    
//    for(unsigned int i=1;i<=octaves;i++){
//        std::cout << "OCTAVE NUMBER " << i <<  std::endl;
//        resolution = starting_resolution*pow(2,i-1);
//        kp_resolution = starting_kp_grid_reso*i;
//        descr_rad = starting_descr_rad*i;
        
//        for(unsigned int j=0;j<=interval;j++){
//            std::cout << "INTERVAL NUMBER " << j <<  std::endl;
//            typename pcl::PointCloud<PointType>::Ptr subspace(new typename pcl::PointCloud<PointType>());
//            float subresolution = resolution;
//            float sub_kp_resolution = kp_resolution;
//            float sub_descr_rad = descr_rad;
//            pcl::UniformSampling<PointType> sampling;
            
//            subresolution += (j+1)*resolution/(interval+1);
//            sub_kp_resolution += (j+1)*kp_resolution/(interval+1);
//            sub_descr_rad += (j+1)*descr_rad/(interval+1);
            
//            sampling.setInputCloud(normalized_input);
//            sampling.setRadiusSearch (subresolution);
//            sampling.filter(*subspace);
            
//            _keypoints->push_back(compute_keypoints(normalized_input, sub_kp_resolution, min, max));
//            pcl::PointCloud<DescriptorType>::Ptr desc = compute_descriptor(subspace,_keypoints->back(),sub_descr_rad);
//            _descriptors->push_back(desc);
//        }
//    }
//    height_ = _descriptors->size();
    
//}

GSHOTPyramid::~GSHOTPyramid()
{
//    descriptors_->clear();
//    keypoints_.clear();
//    height_ = 0;
}


/*
 * WARNING : need to build a sub structure to partially specialize the pyramid
 */
DescriptorsPtr
GSHOTPyramid::compute_descriptor(PointCloudPtr input, PointCloudPtr keypoints, float descr_rad)
{
    DescriptorsPtr descriptors (new Descriptors());
    SurfaceNormalsPtr normals (new SurfaceNormals());

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

    for (size_t i = 0; i < descriptors->size(); ++i){
        for (size_t j = 0; j < DescriptorType::descriptorSize(); ++j){

            if (pcl_isnan(descriptors->points[i].descriptor[j])){
                descriptors->points[i].descriptor[j] = 0;
            }
        }

    }
    
    return descriptors;
}

PointCloudPtr
GSHOTPyramid::compute_keypoints(PointCloudPtr input, float grid_reso, PointType min, PointType max, int index){
    
    int pt_nb_x = (int)((max.x-min.x)/grid_reso+1);
    int pt_nb_y = (int)((max.y-min.y)/grid_reso+1);
    int pt_nb_z = (int)((max.z-min.z)/grid_reso+1);
    int pt_nb = pt_nb_x*pt_nb_y*pt_nb_z;
    
    Eigen::Vector3i topo = Eigen::Vector3i(pt_nb_z, pt_nb_y, pt_nb_x);
    topology[index] = topo;
    
    PointCloudPtr keypoints (new PointCloudT (pt_nb,1,PointType()));
    

#if defined(_OPENMP)
#pragma omp parallel for num_threads(omp_get_max_threads())
#endif
    for(int i=0;i<pt_nb_z;++i){
        for(int j=0;j<pt_nb_y;++j){
            for(int k=0;k<pt_nb_x;++k){
                PointType p = PointType();
                p.x = min.x + k*grid_reso;
                p.y = min.y + j*grid_reso;
                p.z = min.z + i*grid_reso;
                keypoints->at(k + j * pt_nb_x + i * pt_nb_y * pt_nb_x) = p;
//                cout << k + j * pt_nb_x + i * pt_nb_y * pt_nb_x << " / " << pt_nb << endl;
            }
        }
    }
    
    return keypoints;
}


double GSHOTPyramid::computeCloudResolution (PointCloudConstPtr cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}


//Tensor3DF GSHOTPyramid::TensorMap(Level & level){
//    Tensor3DF res( level.depths(), level.rows(), level.cols() * DescriptorSize);
//    res() = Eigen::TensorMap< Eigen::Tensor< Scalar, 3, Eigen::RowMajor> >(level().data()->data(),
//                                                                          level.depths(), level.rows(),
//                                                                          level.cols() * DescriptorSize);
//    return res;
//}

Tensor3DF GSHOTPyramid::TensorMap(Level level){
    const Tensor3DF res( Eigen::TensorMap< Eigen::Tensor< Scalar, 3, Eigen::RowMajor> >(level().data()->data(),
                                                                   level.depths(), level.rows(),
                                                                   level.cols() * DescriptorSize));
    return res;
}


///** GETTER AND SETTERS **/
//#pragma mark -  GETTERS AND SETTER METHODS

//pcl::PointCloud<DescriptorType> GSHOTPyramid::get_descriptors_layer(unsigned int i)
//{
//    return *(_descriptors->at(i));
//}

//pcl::PointCloud<PointType> GSHOTPyramid::get_keypoints_layer(unsigned int i)
//{
//    return *(_keypoints->at(i));
//}

int GSHOTPyramid::interval() const
{
    return interval_;
}

Eigen::Vector3i GSHOTPyramid::pad() const
{
    return pad_;
}

bool GSHOTPyramid::empty() const
{
    return levels_.empty();
}

const vector<GSHOTPyramid::Level> & GSHOTPyramid::levels() const{
    
    return levels_;
}

const vector<float> & GSHOTPyramid::resolutions() const{

    return resolutions_;
}

//int GSHOTPyramid::get_octaves()
//{
//    return _octaves;
//}
//int GSHOTPyramid::get_sub_between_octaves()
//{
//    return (_descriptors->size()-1)/(_octaves+1);
//}

//int GSHOTPyramid::get_height()
//{
//    return height_;
//}

//float GSHOTPyramid::get_original_resolution()
//{
//    return _original_resolution;
//}

//Eigen::Vector3i GSHOTPyramid::getLayerTopology(int i){
    
//    return topology[i];
//}

//float GSHOTPyramid::get_layer_resolution(int i)
//{
//    int oct = (int) (i/get_octaves());
//    int pos_in_octave = i%get_octaves();
//    float dist_btw_octave = _original_resolution/pow(2,oct) - _original_resolution/pow(2,oct+1);
//    return _original_resolution/pow(2,oct) - dist_btw_octave*pos_in_octave/(get_sub_between_octaves()+1);
//}
///*
// * Need to declare every descriptor we intend to use
// */
//const char* GSHOTPyramid::get_descriptor_type()
//{
//    //return typeid(DescriptorType).name();
//    return "SHOT352";
//}

//const char* GSHOTPyramid::get_point_type()
//{
//    //return typeid(PointType).name();
//    return "XRZRGB";
//}



//void GSHOTPyramid::toString()
//{
    
//    std::cout<<"Pyramid specifications :"<<std::endl<<std::endl;
//    std::cout<<"Descriptor type : "<<this->get_descriptor_type()<<std::endl;
//    std::cout<<"Point type : "<<this->get_point_type()<<std::endl;
//    std::cout<<"Number of octaves : "<<this->_octaves<<std::endl;
//    std::cout<<"Height : "<<this->_descriptors->size()<<" layers ("<<get_sub_between_octaves()<<" subsections between octaves)"<<std::endl;
//    if(_original_resolution==0) {
//        std::cout<<"No presampling used, pyramid starting at the original resolution."<<std::endl;
//    }
//    else {
//        std::cout << "Resolution of the first floor : "<<_original_resolution<<std::endl;
//    }
//}





