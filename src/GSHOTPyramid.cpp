// Written by Fisichella Thomas
// Date 25/05/2018

#include "GSHOTPyramid.h"

using namespace Eigen;
using namespace FFLD;
using namespace std;


const int GSHOTPyramid::DescriptorSize;

GSHOTPyramid::GSHOTPyramid() : pad_( Eigen::Vector3i(0, 0, 0)), interval_(0)
{
}

GSHOTPyramid::GSHOTPyramid(const PointCloudPtr input, Eigen::Vector3i pad, int interval, float starting_resolution,
                           int nbOctave):
pad_( Eigen::Vector3i(0, 0, 0)), interval_(0)
{
    if (input->empty() || (pad.x() < 1) || (pad.y() < 1) || (pad.z() < 1) || (interval < 1)) {
        cerr << "Attempting to create an empty pyramid" << endl;
        return;
    }
    
    pad_ = pad;
    interval_ = interval;
    
    float resolution;
    cout << "GSHOTPyr::constructor starting_resolution : "<<starting_resolution<<endl;


    levels_.resize( interval_ * nbOctave);
    resolutions_.resize( interval_ * nbOctave);
    keypoints_.resize( interval_ * nbOctave);
    topology.resize( interval_ * nbOctave);
    
    PointType minTmp;
    PointType min;
    PointType max;
    pcl::getMinMax3D(*input, minTmp, max);

    //TODO : res/2 make it works only for interval = 1 see Mixture::posLatentSearch
    sceneOffset_ = Vector3i(floor(minTmp.z/starting_resolution/2.0),
                            floor(minTmp.y/starting_resolution/2.0),
                            floor(minTmp.x/starting_resolution/2.0));

    min.x = floor(minTmp.x/starting_resolution)*starting_resolution;
    min.y = floor(minTmp.y/starting_resolution)*starting_resolution;
    min.z = floor(minTmp.z/starting_resolution)*starting_resolution;

    PointCloudPtr subspace(new PointCloudT());
    pcl::UniformSampling<PointType> sampling;
    sampling.setInputCloud(input);
    sampling.setRadiusSearch (starting_resolution);
    sampling.filter(*subspace);

#pragma omp parallel for
    for (int i = 0; i < interval_; ++i) {
#pragma omp parallel for
        for (int j = 0; j < nbOctave; ++j) {
            int index = i + j * interval_;
            resolution = starting_resolution / pow(2.0, -static_cast<double>(i) / interval) * pow(2.0, j);

            resolutions_[index] = resolution;

            cout << "GSHOTPyr::constructor radius resolution at lvl "<<i<<" = "<<resolution<<endl;
            cout << "GSHOTPyr::constructor lvl size : "<<subspace->size()<<endl;
            cout << "GSHOTPyr::constructor index "<<index<<endl;

            keypoints_[index] = compute_keypoints(resolution, min, max, index);
            DescriptorsPtr descriptors = compute_descriptor(subspace, keypoints_[index], 2*resolution);


            Level level( topology[index](0), topology[index](1), topology[index](2));
            int kpt = 0;
            for (int z = 0; z < level.depths(); ++z){
                for (int y = 0; y < level.rows(); ++y){
                    for (int x = 0; x < level.cols(); ++x){
                        for( int k = 0; k < GSHOTPyramid::DescriptorSize; ++k){
                            level()(z, y, x)(k) = descriptors->points[kpt].descriptor[k];
                        }
                        ++kpt;
                    }
                }
            }

            //Once the first level is done, push it to the array of level
            levels_[index] = level;
            cout<<"GSHOTPyramid::constr dims level "<<index<<" : " <<level().dimension(0)<<" / " << level().dimension(1)
              <<" / " << level().dimension(2)<< endl;
        }
    }
}


void GSHOTPyramid::sumConvolve(const Level & filter, vector<Tensor3DF >& convolutions) const
{

    convolutions.resize(levels_.size());
    Level filt = filter.agglomerate();

//#pragma omp parallel for num_threads(2)
    for (int i = 0; i < levels_.size(); ++i){
        cout<<"GSHOTPyramid::sumConvolve filter.size() : "<< filter.size()
           << " with levels_[" <<i<< "].size() : " << levels_[i].size() << endl;

        if ((levels_[i]().dimension(0) < filter().dimension(0)) || (levels_[i]().dimension(1) < filter().dimension(1) )
                || (levels_[i]().dimension(2) < filter().dimension(2) )) {
            cout<<"GSHOTPyramid::sumConvolve error : " <<levels_[i]().dimension(0) - filter().dimension(0)+1<<" < "<<filt().dimension(0)
               <<" / " << levels_[i]().dimension(1) - filter().dimension(1)+1<<" < "<<filt().dimension(1)
              <<" / " << levels_[i]().dimension(2) - filter().dimension(2)+1<<" < "<<filt().dimension(2)<< endl;
            return;
        } else{
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
                    }
                }
            }



            Convolve(lvl, filt, convolutions[i]);
        }
    }
}


void GSHOTPyramid::convolve(const Level & filter, vector<Tensor3DF >& convolutions) const
{

    convolutions.resize(levels_.size());

//#pragma omp parallel for
    for (int i = 0; i < levels_.size(); ++i){
        cout<<"GSHOTPyramid::convolve filter.size() : "<< filter.size()
           << " with levels_[" <<i<< "].size() : " << levels_[i].size() << endl;
        Convolve(levels_[i], filter, convolutions[i]);
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


    convolution = level.convolve(filter);

//    convolution = level.khi2Convolve(filter);

//    convolution = level.EMD(filter);


    cout<<"GSHOTPyramid::convolve results.depths() : "<< convolution.depths() << endl;
    cout<<"GSHOTPyramid::convolve results.rows() : "<< convolution.rows() << endl;
    cout<<"GSHOTPyramid::convolve results.cols() : "<< convolution.cols() << endl;

    cout<<"GSHOTPyramid::convolve results.max() : "<< convolution.max() << endl;
    cout<<"GSHOTPyramid::convolve filter.max() : "<< TensorMap(filter).max() << endl;
    cout<<"GSHOTPyramid::convolve filter.norm() : "<< filter.lvlSquaredNorm() << endl;


}


//GSHOTPyramid::~GSHOTPyramid()
//{
////    keypoints_.clear();
//}

std::vector<float> GSHOTPyramid::minMaxScaler(std::vector<float> data){
    std::vector<float> result_min_max(data.size());

    float sum = 0;
    for (int i = 0; i < data.size(); i++){
        sum += data.at(i);
    }


    for (int i = 0; i < data.size(); i++){
        if( sum != 0) result_min_max[i] = data.at(i) /sum;
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
    

#pragma omp parallel for
    for(int z=0;z<pt_nb_z;++z){
#pragma omp parallel for
        for(int y=0;y<pt_nb_y;++y){
#pragma omp parallel for
            for(int x=0;x<pt_nb_x;++x){
                PointType p = PointType();
                p.x = min.x + x*grid_reso;
                p.y = min.y + y*grid_reso;
                p.z = min.z + z*grid_reso;
                keypoints->at(x + y * pt_nb_x + z * pt_nb_y * pt_nb_x) = p;
            }
        }
    }
    
    return keypoints;
}


DescriptorsPtr
GSHOTPyramid::compute_descriptor(PointCloudPtr input, PointCloudPtr keypoints, float descr_rad)
{
    DescriptorsPtr descriptors (new Descriptors());
    SurfaceNormalsPtr normals (new SurfaceNormals());

    pcl::NormalEstimationOMP<PointType,NormalType> norm_est;
    norm_est.setKSearch (8);
    norm_est.setInputCloud (input);
    norm_est.compute (*normals);

    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad);
    descr_est.setInputCloud (keypoints);
    descr_est.setInputNormals (normals);
    descr_est.setSearchSurface (input);
    descr_est.compute (*descriptors);

    cout<<"GSHOT:: descriptors size = "<<descriptors->size()<<endl;
    cout<<"GSHOT:: keypoints size = "<<keypoints->size()<<endl;

#pragma omp parallel for
    for (size_t i = 0; i < descriptors->size(); ++i){
        std::vector<float> data_tmp(DescriptorSize);

        if (pcl_isnan(descriptors->points[i].descriptor[0])){
            descriptors->points[i].descriptor[0] = 0;
        }
        for (size_t j = 0; j < DescriptorSize; ++j){

            if (pcl_isnan(descriptors->points[i].descriptor[j])){
                descriptors->points[i].descriptor[j] = 0;
            }

            data_tmp[j] = descriptors->points[i].descriptor[j];
        }
        //normalize descriptor
        std::vector<float> value_descriptor_scaled = minMaxScaler(data_tmp);

//        float sum = 0;
        for (size_t j = 0; j < DescriptorSize; ++j){
            descriptors->points[i].descriptor[j] = value_descriptor_scaled.at(j);
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

Tensor3DF GSHOTPyramid::TensorMap(Level level){
    const Tensor3DF res( Eigen::TensorMap< Eigen::Tensor< Scalar, 3, Eigen::RowMajor> >(level().data()->data(),
                                                                   level.depths(), level.rows(),
                                                                   level.cols() * DescriptorSize));
    return res;
}

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

//Read point cloud from a path
/*static*/ int FFLD::readPointCloud(std::string object_path, PointCloudPtr point_cloud){
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

