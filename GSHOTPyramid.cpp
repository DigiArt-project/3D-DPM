
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
//    starting_resolution = 0.09;//25 * computeCloudResolution(input);//0.00338117;//
    cout << "GSHOTPyr::constructor starting_resolution : "<<starting_resolution<<endl;
//    float kp_resolution;
//    float descr_rad;


    levels_.resize( interval_ * nbOctave);
    resolutions_.resize( interval_ * nbOctave);
    keypoints_.resize( interval_ * nbOctave);
    topology.resize( interval_ * nbOctave);
    
    PointType minTmp;
    PointType min;
    PointType max;
    pcl::getMinMax3D(*input, minTmp, max);

    min.x = floor(minTmp.x/starting_resolution)*starting_resolution;
    min.y = floor(minTmp.y/starting_resolution)*starting_resolution;
    min.z = floor(minTmp.z/starting_resolution)*starting_resolution;



    //TODO #pragma omp parallel for i
    for (int i = 0; i < interval_; ++i) {
        for (int j = 0; j < nbOctave; ++j) {
            int index = i + j * interval_;
//            resolution = 10 * starting_resolution * pow(2.0, -static_cast<double>(i) / interval) / pow(2.0, j);
            resolution = starting_resolution / pow(2.0, -static_cast<double>(i) / interval) * pow(2.0, j);



//            PointCloudPtr subspace(new PointCloudT());
//            pcl::UniformSampling<PointType> sampling;
//            sampling.setInputCloud(input);
//            sampling.setRadiusSearch (resolution);
//            sampling.filter(*subspace);

            resolutions_[index] = resolution;
//            pcl::VoxelGrid<PointCloudT> sor;
//            sor.setInputCloud (input);
//            sor.setLeafSize (resolution, resolution, resolution);
//            sor.filter (*subspace);

            cout << "GSHOTPyr::constructor radius resolution at lvl "<<i<<" = "<<resolution<<endl;
            cout << "GSHOTPyr::constructor lvl size : "<<input->size()<<endl;
            cout << "GSHOTPyr::constructor index "<<index<<endl;

            keypoints_[index] = compute_keypoints(resolution, min, max, index);
            DescriptorsPtr descriptors = compute_descriptor(input, keypoints_[index], 2*resolution);


//            Level level( topology[index](0), topology[index](1), topology[index](2));
//            Cell* levelCell = &(level()(0));
//            //Loop over the number of keypoints available
//            for (int kpt = 0; kpt < keypoints_[index]->size(); ++kpt){
//                //For each keypoint, create a cell which will contain the corresponding shot descriptor
//                Cell cell(GSHOTPyramid::DescriptorSize);
//                //Then for each value of the associated descriptor, fill in the cell
//                for( int k = 0; k < GSHOTPyramid::DescriptorSize; ++k){
//                    cell(k) = descriptors->points[kpt].descriptor[k];
//                    if(descriptors->points[kpt].descriptor[k]<0){
//                        cout << "GSHOTPyr::constructor descriptors->points["<<kpt<<"].descriptor["<<k<<"] = "
//                             << descriptors->points[kpt].descriptor[k] << endl;
//                    }
//                    if(cell(k)<0){
//                        cout << "GSHOTPyr::constructor cell["<<k<<"] = "
//                             << cell(k) << endl;
//                    }
//    //                cout << "GSHOTPyr::constructor cell.row("<<k<<") "
//    //                     << cell.row(k) << endl;
//                }
//               //Add the cell to the current level
//                //TODO check the order of the cells
//                *levelCell = cell;
//                for( int k = 0; k < GSHOTPyramid::DescriptorSize; ++k){

//                    if((*levelCell)(k)<0){
//                        cout << "GSHOTPyr::constructor *levelCell["<<k<<"] = "
//                             << (*levelCell)(k) << endl;
//                    }
//                }
//                ++levelCell;
//            }


            Level level( topology[index](0), topology[index](1), topology[index](2));
            int kpt = 0;
//#pragma omp parallel for
            for (int z = 0; z < level.depths(); ++z){
                for (int y = 0; y < level.rows(); ++y){
                    for (int x = 0; x < level.cols(); ++x){
                        for( int k = 0; k < GSHOTPyramid::DescriptorSize; ++k){
                            level()(z, y, x)(k) = descriptors->points[kpt].descriptor[k];
//                            if(descriptors->points[kpt].descriptor[k]<0){
//                                cout << "GSHOTPyr::constructor descriptors->points["<<kpt<<"].descriptor["<<k<<"] = "
//                                     << descriptors->points[kpt].descriptor[k] << endl;
//                            }
                        }
                        ++kpt;
                    }
                }
            }


            cout << "GSHOTPyr::constructor kpt = "<<kpt<< endl;


            //Once the first level is done, push it to the array of level
            levels_[index] = level;
            cout<<"GSHOTPyramid::constr dims level "<<index<<" : " <<level().dimension(0)<<" / " << level().dimension(1)
              <<" / " << level().dimension(2)<< endl;
            cout<<"GSHOTPyramid::constr level hasNegValue : "<< levels_[index].hasNegValue() << endl;
        }
    }
}


void GSHOTPyramid::sumConvolve(const Level & filter, vector<Tensor3DF >& convolutions) const
{

    convolutions.resize(levels_.size());
//    Tensor3DF filt = TensorMap( filter).agglomerate(DescriptorSize);
    Level filt = filter.agglomerate();


//    cout<<"GSHOT::sumConvolve filter max Value = "<<TensorMap( filter)().maximum()<<endl;
//    cout<<"GSHOT::sumConvolve filt max Value = "<<filt().maximum()<<endl;

//    cout<<"GSHOT::sumConvolve filter = "<<filt()<<endl;

//#pragma omp parallel for num_threads(2)
    for (int i = 0; i < levels_.size(); ++i){
        cout<<"GSHOTPyramid::convolve filter.size() : "<< filter.size()
           << " with levels_[" <<i<< "].size() : " << levels_[i].size() << endl;
//        cout<<"GSHOT::sumConvolve lvl = ";

        Level lvl( levels_[i].depths() - filter.depths() + 1,
                   levels_[i].rows() - filter.rows() + 1,
                   levels_[i].cols() - filter.cols() + 1);
        for (int z = 0; z < lvl.depths(); ++z) {
            for (int y = 0; y < lvl.rows(); ++y) {
                for (int x = 0; x < lvl.cols(); ++x) {
                    lvl()(z, y, x) = levels_[i].agglomerateBlock(z, y, x,
                                                            filter.depths(),
                                                            filter.rows(),
                                                            filter.cols())()(0,0,0);
//                        cout<< lvl()(z, y, x)(j) << " ";

//                     cout << endl;
                }
            }
        }



        Convolve(/*TensorMap(*/lvl/*)*/, filt, convolutions[i]);
    }
}


void GSHOTPyramid::convolve(const Level & filter, vector<Tensor3DF >& convolutions) const
{

    convolutions.resize(levels_.size());

//#pragma omp parallel for
    for (int i = 0; i < levels_.size(); ++i){
        cout<<"GSHOTPyramid::convolve filter.size() : "<< filter.size()
           << " with levels_[" <<i<< "].size() : " << levels_[i].size() << endl;
//        cout<<"GSHOTPyramid::convolve filter.cols() : "<< filter.cols()
//           << " with levels_[" <<i<< "].cols() : " << levels_[i].cols() << endl;
//        cout<<"GSHOTPyramid::convolve filter.cols() : "<< TensorMap(filter).cols()
//           << " with levels_[" <<i<< "].cols() : " << TensorMap(levels_[i]).cols() << endl;
        Convolve(/*TensorMap(*/levels_[i]/*)*/, /*TensorMap(*/filter/*)*/, convolutions[i]);
    }
}

void GSHOTPyramid::Convolve(const Level & level, const Level & filter, Tensor3DF & convolution)
{
    // Nothing to do if x is smaller than y
    if ((level().dimension(0) < filter().dimension(0)) || (level().dimension(1) < filter().dimension(1) )
            || (level().dimension(2) < filter().dimension(2) )) {
        cout<<"GSHOTPyramid::convolve error : level size is smaller than filter" << endl;
        cout<<"GSHOTPyramid::convolve error : " <<level().dimension(0)<<" < "<<filter().dimension(0)
           <<" / " << level().dimension(1)<<" < "<<filter().dimension(1)
          <<" / " << level().dimension(2)<<" < "<<filter().dimension(2)<< endl;
        return;
    }


    cout<<"GSHOTPyramid::convolve level hasNegValue : "<< level.hasNegValue() << endl;
    cout<<"GSHOTPyramid::convolve filter hasNegValue : "<< filter.hasNegValue() << endl;


    convolution = level.convolve(filter);

//    for (int z = 0; z < convolution.depths(); ++z) {
//        for (int y = 0; y < convolution.rows(); ++y) {
//            for (int x = 0; x < convolution.cols(); ++x) {
//                if( convolution()(z, y, x) < 0) cout<<"GSHOTPyramid::convolve convolution hasNegValue" << endl;
//            }
//        }
//    }
    cout<<"GSHOTPyramid::convolve level.depths() : "<< level.depths() << endl;
    cout<<"GSHOTPyramid::convolve level.rows() : "<< level.rows() << endl;
    cout<<"GSHOTPyramid::convolve level.cols() : "<< level.cols() << endl;
    cout<<"GSHOTPyramid::convolve filter.depths() : "<< filter.depths() << endl;
    cout<<"GSHOTPyramid::convolve filter.rows() : "<< filter.rows() << endl;
    cout<<"GSHOTPyramid::convolve filter.cols() : "<< filter.cols() << endl;

    cout<<"GSHOTPyramid::convolve results.depths() : "<< convolution.depths() << endl;
    cout<<"GSHOTPyramid::convolve results.rows() : "<< convolution.rows() << endl;
    cout<<"GSHOTPyramid::convolve results.cols() : "<< convolution.cols() << endl;
}


void GSHOTPyramid::Convolve(const Tensor3DF & level, const Tensor3DF & filter, Tensor3DF & convolution)
{
    // Nothing to do if x is smaller than y
    if ((level().dimension(0) < filter().dimension(0)) || (level().dimension(1) < filter().dimension(1) )
            || (level().dimension(2) < filter().dimension(2) )) {
        cout<<"GSHOTPyramid::convolve error : level size is smaller than filter" << endl;
        cout<<"GSHOTPyramid::convolve error : " <<level().dimension(0)<<" < "<<filter().dimension(0)
           <<" / " << level().dimension(1)<<" < "<<filter().dimension(1)
          <<" / " << level().dimension(2)<<" < "<<filter().dimension(2)<< endl;
        return;
    }

//    cout<<"GSHOTPyramid::convolve dims : " <<level().dimension(0)<<" < "<<filter().dimension(0)
//       <<" / " << level().dimension(1)<<" < "<<filter().dimension(1)
//      <<" / " << level().dimension(2)<<" < "<<filter().dimension(2)<< endl;

//    cout<<"GSHOTPyramid::convolve level : "<< level() << endl;

//    cout<<"GSHOTPyramid::convolve filter : "<< filter() << endl;

    Tensor3DF aux;
//    aux().resize( level.depths() - filter.depths() + 1,
//              level.rows() - filter.rows() + 1,
//              (level.cols()/DescriptorSize - filter.cols()/DescriptorSize + 1)*DescriptorSize);//TODO prob here

    Eigen::array<ptrdiff_t, 3> dims({0, 1, 2});
    aux() = level().convolve(filter(), dims);

    cout<<"GSHOTPyramid::convolve aux.depths() : "<< aux.depths() << endl;
    cout<<"GSHOTPyramid::convolve aux.rows() : "<< aux.rows() << endl;
    cout<<"GSHOTPyramid::convolve aux.cols() : "<< aux.cols() << endl;

//    Eigen::array<Eigen::DenseIndex, 3> strides({1, 1, DescriptorSize});
//    convolution() = aux().stride(strides);

    convolution().resize( aux.depths(), aux.rows(), aux.cols()/DescriptorSize + 1);

    int cpt = 0;
    for (int i = 0; i < aux.depths(); ++i) {
        for (int j = 0; j < aux.rows(); ++j) {
            cpt=0;
            for (int k = 0; k < aux.cols(); ++k) {
                if( k % DescriptorSize == 0 && cpt < convolution.cols()) {
//                    cout<<"k = "<<k<<endl;
                    convolution()(i,j,cpt) = aux()(i,j,k)/*/sqrt(filter.squaredNorm())*/;
                    ++cpt;
                }
            }
        }
    }
    cout<<"GSHOTPyramid::convolve cpt : "<< cpt << endl;
    cout<<"GSHOTPyramid::convolve results.depths() : "<< convolution.depths() << endl;
    cout<<"GSHOTPyramid::convolve results.rows() : "<< convolution.rows() << endl;
    cout<<"GSHOTPyramid::convolve results.cols() : "<< convolution.cols() << endl;
}

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



std::vector<float> GSHOTPyramid::minMaxScaler(std::vector<float> data, float max, float min){
    std::vector<float> result_min_max(data.size());

//    cout<<"GSHOTPyramid::result max : "<< max << endl;

//    cout<<"GSHOTPyramid::result min : "<< min << endl;

    if(max == min) return result_min_max;

    float sum = 0;
    for (int i = 0; i < data.size(); i++){
        sum+=(data.at(i) - min)/(max - min);
    }


//    cout<<"GSHOTPyramid::result_min_max sum : "<< sum << endl;

    for (int i = 0; i < data.size(); i++){
        result_min_max[i] = (data.at(i) - min)/(max - min)/sum;
//        cout<<"GSHOTPyramid::result_min_max[i] : "<< result_min_max[i] << endl;
    }

    return result_min_max;
}




PointCloudPtr
GSHOTPyramid::compute_keypoints(float grid_reso, PointType min, PointType max, int index){

    int pt_nb_x = ceil((max.x-min.x)/grid_reso+1);
    int pt_nb_y = ceil((max.y-min.y)/grid_reso+1);
    int pt_nb_z = ceil((max.z-min.z)/grid_reso+1);

    int pt_nb = pt_nb_x*pt_nb_y*pt_nb_z;
    
    Eigen::Vector3i topo = Eigen::Vector3i(pt_nb_z, pt_nb_y, pt_nb_x);
    topology[index] = topo;
    
    PointCloudPtr keypoints (new PointCloudT (pt_nb,1,PointType()));
    

//#pragma omp parallel for num_threads(omp_get_max_threads())
    for(int z=0;z<pt_nb_z;++z){
        for(int y=0;y<pt_nb_y;++y){
            for(int x=0;x<pt_nb_x;++x){
                PointType p = PointType();
                p.x = min.x + x*grid_reso;
                p.y = min.y + y*grid_reso;
                p.z = min.z + z*grid_reso;
                keypoints->at(x + y * pt_nb_x + z * pt_nb_y * pt_nb_x) = p;
//                cout << k + j * pt_nb_x + i * pt_nb_y * pt_nb_x << " / " << pt_nb << endl;
            }
        }
    }
    
    return keypoints;
}

/*
 * WARNING : need to build a sub structure to partially specialize the pyramid
 */
DescriptorsPtr
GSHOTPyramid::compute_descriptor(PointCloudPtr input, PointCloudPtr keypoints, float descr_rad)
{
    DescriptorsPtr descriptors (new Descriptors());
    SurfaceNormalsPtr normals (new SurfaceNormals());

    pcl::NormalEstimation<PointType,NormalType> norm_est;
    norm_est.setKSearch (3);
    norm_est.setInputCloud (input);
    norm_est.compute (*normals);

    pcl::SHOTEstimation<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad);
    descr_est.setInputCloud (keypoints);
    descr_est.setInputNormals (normals);
    descr_est.setSearchSurface (input);
    descr_est.compute (*descriptors);

    cout<<"GSHOT:: descriptors size = "<<descriptors->size()<<endl;
    cout<<"GSHOT:: keypoints size = "<<keypoints->size()<<endl;


    for (size_t i = 0; i < descriptors->size(); ++i){
        std::vector<float> data_tmp(DescriptorSize);

        if (pcl_isnan(descriptors->points[i].descriptor[0])){
            descriptors->points[i].descriptor[0] = 0;
        }
        float min=descriptors->points[i].descriptor[0], max=descriptors->points[i].descriptor[0];
        for (size_t j = 0; j < DescriptorSize; ++j){

            if (pcl_isnan(descriptors->points[i].descriptor[j])){
                descriptors->points[i].descriptor[j] = 0;
            }
            if(descriptors->points[i].descriptor[j]>max) max = descriptors->points[i].descriptor[j];
            if(descriptors->points[i].descriptor[j]<min) min = descriptors->points[i].descriptor[j];


            data_tmp[j] = descriptors->points[i].descriptor[j];
        }
        //normalize descriptor
        std::vector<float> value_descriptor_scaled = minMaxScaler(data_tmp, max, min);

//        float sum = 0;
        for (size_t j = 0; j < DescriptorSize; ++j){
            descriptors->points[i].descriptor[j] = value_descriptor_scaled.at(j);
//            cout<<"GSHOTPyramid::descriptor normalized : "<< descriptors->points[i].descriptor[j] << endl;
//            sum += descriptors->points[i].descriptor[j];
        }
//        cout<<"GSHOTPyramid::sum of the descriptor normalized : "<< sum << endl;

    }

    return descriptors;
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



//Read point cloud from a path

int FFLD::readPointCloud(std::string object_path, PointCloudPtr point_cloud){
    std::string extension = boost::filesystem::extension(object_path);
    if (extension == ".pcd" || extension == ".PCD")
    {
        if (pcl::io::loadPCDFile(object_path.c_str() , *point_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return (-1);
        }
    }
    else if (extension == ".ply" || extension == ".PLY")
    {
        if (pcl::io::loadPLYFile(object_path.c_str() , *point_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return (-1);
        }
    }
    else
    {
        std::cout << "\n file extension is not correct." << std::endl;
        return -1;
    }
    return 1;

}

