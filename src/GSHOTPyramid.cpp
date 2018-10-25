// Written by Fisichella Thomas
// Date 25/05/2018

#include "GSHOTPyramid.h"

using namespace Eigen;
using namespace FFLD;
using namespace std;


const int GSHOTPyramid::DescriptorSize;

GSHOTPyramid::GSHOTPyramid() : pad_( Vector3i(0, 0, 0)), interval_(0)
{
}

GSHOTPyramid::GSHOTPyramid(const PointCloudPtr input, Vector3i filterSizes,
//                           originalOrientation,
                           int interval, float starting_resolution, int thresh,
                           int nbOctave):
    interval_(0), filterSizes_(filterSizes)
{
    if (input->empty() || (interval < 1)) {
        cerr << "Attempting to create an empty pyramid" << endl;
        return;
    }
    
    interval_ = interval;
    
    float resolution;
//    cout << "GSHOTPyr::constructor starting_resolution : "<<starting_resolution<<endl;


    levels_.resize( interval_ * nbOctave);
    resolutions_.resize( interval_ * nbOctave);
    keypoints_.resize( interval_ * nbOctave);
    rectangles_.resize( interval_ * nbOctave);
    topology_.resize( interval_ * nbOctave);

    for (int j = 0; j < interval_; ++j) {
        for (int k = 0; k < nbOctave; ++k) {
            int lvl = j + k * interval_;
            resolutions_[lvl] = starting_resolution / pow(2.0, -static_cast<double>(j) / interval) * pow(2.0, k);
        }
    }
    
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

//    float orientationFrom[9] = {0,0,1,1,0,0,0,1,0};
//    float orientationFrom[9] = {1,0,0,0,1,0,0,0,1};
//    float orientationFrom[9] = {-0.118525, 0.931382, 0.344209, -0.992036, -0.125949, -0.000797627, 0.0426101, -0.341563, 0.938893};//{1,0,0,0,1,0,0,0,1};
//    float orientationFrom[9] = {-0.132104, -0.589017, 0.79725, -0.0987371, 0.808118, 0.580686, -0.986306, -0.00200695, -0.164914};
//chair    float orientationFrom[9] = {-0.199227, -0.865065, 0.460403, -0.599213, -0.264214, -0.755734, 0.775404, -0.426442, -0.46572};
    //bed using min filterSize for GSHOT, works good when direction is correct but can be easily wrong
//    float orientationFrom[9] = {-0.131707, -0.139466, 0.981429, 0.941407, 0.292508, 0.167903, -0.310492, 0.946038, 0.0927687};
    //bed using mean filterSize for GSHOT, orientation more robust but less accurate
    float orientationFrom[9] = {-0.330336, 0.0100554, 0.94381, 0.897447, 0.313065, 0.310774, -0.292348, 0.949679, -0.112441};

    resolution = starting_resolution * 2;

    globalKeyPts = computeKeyptsWithThresh(subspace, resolution, min, max, filterSizes, thresh);
    globalDescriptors = compute_descriptor(subspace, globalKeyPts,
//                                           std::min(filterSizes(0), std::min(filterSizes(1), filterSizes(2)))*resolution);
                                        filterSizes.sum()*resolution/3.0);

    for(int i=0;i<levels_.size();++i){
        levels_[i].resize( globalKeyPts->size());
        keypoints_[i].resize( globalKeyPts->size());
        rectangles_[i].resize( globalKeyPts->size());
        for(int j=0;j<levels_[i].size();++j){
            PointCloudPtr cloud (new PointCloudT);
            keypoints_[i][j] = cloud;
        }

    }
    cout << "GSHOTPyr::constructor globalKeyPts->size() : "<<globalKeyPts->size()<<endl;

    vector<PointCloudPtr> boxKeyPts(2);//[lvl]
    PointType boxStart = PointType();
    PointType boxEnd = PointType();
    boxEnd.x = filterSizes(2)*starting_resolution * 2;
    boxEnd.y = filterSizes(1)*starting_resolution * 2;
    boxEnd.z = filterSizes(0)*starting_resolution * 2;
    PointCloudPtr tmp = compute_keypoints(starting_resolution, boxStart, boxEnd, 0);
    boxKeyPts[0] = tmp;
    tmp = compute_keypoints(resolution, boxStart, boxEnd, 1);
    boxKeyPts[1] = tmp;

    Vector3f origin(0,0,0);
    Vector3f recSize(filterSizes(0) * starting_resolution * 2,
                     filterSizes(1) * starting_resolution * 2,
                     filterSizes(2) * starting_resolution * 2);

    int cpt = 0;
    //for each boxes i
#pragma omp parallel for
    for( int i = 0; i < globalDescriptors->size(); ++i){

        Vector3f boxOrigin(
                    filterSizes(2) * starting_resolution * 2 / 2.0,
                    filterSizes(1) * starting_resolution * 2 / 2.0,
                    filterSizes(0) * starting_resolution * 2 / 2.0);
        //translation from low front left anchor (0,0,0)
        Vector3f translation(
                    globalKeyPts->points[i].x - filterSizes(2) * starting_resolution * 2 / 2.0,
                    globalKeyPts->points[i].y - filterSizes(1) * starting_resolution * 2 / 2.0,
                    globalKeyPts->points[i].z - filterSizes(0) * starting_resolution * 2 / 2.0);

        //TODO check translation
            Eigen::Matrix4f transform = getNormalizeTransform(orientationFrom,
                                                              globalDescriptors->points[i].rf,
                                                              boxOrigin, translation);
//        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
//            cout << "GSHOTPyr::constructor rotation : "<<endl<<transform<<endl;

    //    #pragma omp parallel for
            for (int j = 0; j < interval_; ++j) {
    //    #pragma omp parallel for
                for (int k = 0; k < nbOctave; ++k) {
                    int lvl = j + k * interval_;

//                    PointCloudPtr keypointsBox = compute_keypoints(resolution, globalKeyPts->points[i], p, lvl);
//                    cout << "GSHOTPyr::constructor compute_keypoints done"<<endl;

                    pcl::transformPointCloud (*boxKeyPts[lvl], *(keypoints_[lvl][i]), transform);
                    rectangles_[lvl][i] = Rectangle( origin, recSize, transform);
//                    cout << "GSHOTPyr::constructor transformPointCloud done"<<endl;
//                    cout<<"Keypts1 : "<<keypoints_[lvl][i]->points[0]<<endl;
//                    cout<<"Keypts2 : "<<keypoints_[lvl][i]->points[1]<<endl;

                    DescriptorsPtr descriptors = compute_descriptor(subspace, keypoints_[lvl][i], 2*resolutions_[lvl]);
//                    cout << "GSHOTPyr::constructor compute_descriptor done"<<endl;

                    Level level( topology_[lvl](0), topology_[lvl](1), topology_[lvl](2));
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
                    levels_[lvl][i] = level;
//                    cout << "GSHOTPyr::constructor fillLevel done"<<endl;

                }
            }
//            cout<<"Keypts : "<<keypoints_[1][i]->points[0]<<endl;

        ++cpt;
    }
}


void GSHOTPyramid::sumConvolve(const Level & filter, vector<Tensor3DF >& convolutions) const
{

//    convolutions.resize(levels_.size());
//    Level filt = filter.agglomerate();

////#pragma omp parallel for num_threads(2)
//    for (int i = 0; i < levels_.size(); ++i){
//        cout<<"GSHOTPyramid::sumConvolve filter.size() : "<< filter.size()
//           << " with levels_[" <<i<< "].size() : " << levels_[i].size() << endl;

//        if ((levels_[i]().dimension(0) < filter().dimension(0)) || (levels_[i]().dimension(1) < filter().dimension(1) )
//                || (levels_[i]().dimension(2) < filter().dimension(2) )) {
//            cout<<"GSHOTPyramid::sumConvolve error : " <<levels_[i]().dimension(0) - filter().dimension(0)+1<<" < "<<filt().dimension(0)
//               <<" / " << levels_[i]().dimension(1) - filter().dimension(1)+1<<" < "<<filt().dimension(1)
//              <<" / " << levels_[i]().dimension(2) - filter().dimension(2)+1<<" < "<<filt().dimension(2)<< endl;
//            return;
//        } else{
//            Level lvl( levels_[i].depths() - filter.depths() + 1,
//                       levels_[i].rows() - filter.rows() + 1,
//                       levels_[i].cols() - filter.cols() + 1);

//            for (int z = 0; z < lvl.depths(); ++z) {
//                for (int y = 0; y < lvl.rows(); ++y) {
//                    for (int x = 0; x < lvl.cols(); ++x) {
//                        lvl()(z, y, x) = levels_[i].agglomerateBlock(z, y, x,
//                                                                filter.depths(),
//                                                                filter.rows(),
//                                                                filter.cols())()(0,0,0);
//                    }
//                }
//            }



//            Convolve(lvl, filt, convolutions[i]);
//        }
//    }
}


void GSHOTPyramid::convolve(const Level & filter, vector<vector<Tensor3DF > >& convolutions) const
{
    cout<<"GSHOTPyramid::convolve ..."<<endl;

    convolutions.resize(levels_.size());

    for (int i = 0; i < levels_.size(); ++i){
        convolutions[i].resize(levels_[i].size());
//        cout<<"GSHOTPyramid::convolve at lvl : "<< i << endl;

    #pragma omp parallel for //for each box
        for (int j = 0; j < levels_[i].size(); ++j){
            Convolve(levels_[i][j], filter, convolutions[i][j]);
        }
    }
}

void GSHOTPyramid::Convolve(const Level & level, const Level & filter, Tensor3DF & convolution)
{
    // Nothing to do if x is smaller than y
    if ((level().dimension(0) < filter().dimension(0)) || (level().dimension(1) < filter().dimension(1) )
            || (level().dimension(2) < filter().dimension(2) )) {
//        cout<<"GSHOTPyramid::convolve error : level size is smaller than filter" << endl;
//        cout<<"GSHOTPyramid::convolve error : " <<level().dimension(0)<<" < "<<filter().dimension(0)
//           <<" / " << level().dimension(1)<<" < "<<filter().dimension(1)
//          <<" / " << level().dimension(2)<<" < "<<filter().dimension(2)<< endl;
        return;
    }


    convolution = level.convolve(filter);

//    convolution = level.khi2Convolve(filter);

//    convolution = level.EMD(filter);

//    cout<<"GSHOTPyramid::convolve results.size() : "<< convolution.size() << endl;


//    cout<<"GSHOTPyramid::convolve results.depths() : "<< convolution.depths() << endl;
//    cout<<"GSHOTPyramid::convolve results.rows() : "<< convolution.rows() << endl;
//    cout<<"GSHOTPyramid::convolve results.cols() : "<< convolution.cols() << endl;

//    cout<<"GSHOTPyramid::convolve results.max() : "<< convolution.max() << endl;
//    cout<<"GSHOTPyramid::convolve filter.max() : "<< TensorMap(filter).max() << endl;
//    cout<<"GSHOTPyramid::convolve filter.norm() : "<< filter.lvlSquaredNorm() << endl;


}


Matrix4f GSHOTPyramid::getNormalizeTransform(float* orientationFrom, float* orientationTo,
                                             const Vector3f origin, const Vector3f translation){

    float* finiteOrientation;
    if( std::isfinite(orientationTo[0])){
        finiteOrientation = orientationTo;
    } else{
        finiteOrientation = orientationFrom;
    }

    Matrix3f r0, r1, rotation;
    for(int i=0; i<3;++i){
        for(int j=0; j<3;++j){
            r0(j,i) = orientationFrom[j+i*3];
            r1(j,i) = finiteOrientation[j+i*3];
        }
    }

//    cout<<"r0 : "<<endl<<r0<<endl;
//    cout<<"r1 : "<<endl<<r1<<endl;
    rotation = r1*r0.inverse();
    Eigen::Matrix4f tform;
    tform.setIdentity();
    tform.topLeftCorner (3, 3) = rotation;

    tform.topRightCorner(3, 1) = origin - rotation * origin + translation;
//    cout<<"transform : "<<endl<<tform<<endl;

    return tform;
}


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

    int pt_nb_x = ceil((max.x-min.x)/grid_reso);
    int pt_nb_y = ceil((max.y-min.y)/grid_reso);
    int pt_nb_z = ceil((max.z-min.z)/grid_reso);

    int pt_nb = pt_nb_x*pt_nb_y*pt_nb_z;

    Eigen::Vector3i topo = Eigen::Vector3i(pt_nb_z, pt_nb_y, pt_nb_x);
    topology_[index] = topo;

    PointCloudPtr keypoints (new PointCloudT (pt_nb,1,PointType()));


#pragma omp parallel for
    for(int z=0;z<pt_nb_z;++z){
#pragma omp parallel for
        for(int y=0;y<pt_nb_y;++y){
#pragma omp parallel for
            for(int x=0;x<pt_nb_x;++x){
                PointType p = PointType();
                p.x = min.x + (x+1)*(max.x - min.x) / (pt_nb_x + 1);
                p.y = min.y + (y+1)*(max.y - min.y) / (pt_nb_y + 1);
                p.z = min.z + (z+1)*(max.z - min.z) / (pt_nb_z + 1);
                keypoints->at(x + y * pt_nb_x + z * pt_nb_y * pt_nb_x) = p;
            }
        }
    }

    return keypoints;
}

PointCloudPtr
GSHOTPyramid::computeKeyptsWithThresh(PointCloudPtr cloud, float grid_reso, PointType min, PointType max,
                                      Vector3i filterSizes, int thresh){

    int pt_nb_x = ceil((max.x-min.x)/grid_reso+1)-filterSizes(2);
    int pt_nb_y = ceil((max.y-min.y)/grid_reso+1)-filterSizes(1);
    int pt_nb_z = ceil((max.z-min.z)/grid_reso+1)-filterSizes(0);
    
    PointCloudPtr keypoints (new PointCloudT());
    keypoints->width    = 0;
    keypoints->height   = 1;
    keypoints->points.resize(keypoints->width);

    for(int z=0;z<pt_nb_z;++z){
        for(int y=0;y<pt_nb_y;++y){
            for(int x=0;x<pt_nb_x;++x){
                //put keyPts in the middle of the box
                PointType p = PointType();
                p.x = min.x + (x+filterSizes(2)/2.0)*grid_reso;
                p.y = min.y + (y+filterSizes(1)/2.0)*grid_reso;
                p.z = min.z + (z+filterSizes(0)/2.0)*grid_reso;

                Vector4f ptStart( min.x + x*grid_reso,
                                  min.y + y*grid_reso,
                                  min.z + z*grid_reso, 1);
                Vector4f ptEnd( ptStart(0) + filterSizes(2)*grid_reso,
                                ptStart(1) + filterSizes(1)*grid_reso,
                                ptStart(2) + filterSizes(0)*grid_reso, 1);
                std::vector<int> pt_indices;
                pcl::getPointsInBox(*cloud, ptStart, ptEnd, pt_indices);

                // Discarding regions without enough points
                if(pt_indices.size()>=thresh){
                    keypoints->width    = keypoints->points.size()+1;
                    keypoints->height   = 1;
                    keypoints->points.resize (keypoints->width);
                    keypoints->at(keypoints->points.size()-1) = p;
                }
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

//    cout<<"GSHOT:: descriptors size = "<<descriptors->size()<<endl;
//    cout<<"GSHOT:: keypoints size = "<<keypoints->size()<<endl;

#pragma omp parallel for
    for (size_t i = 0; i < descriptors->size(); ++i){
//        std::vector<float> data_tmp(DescriptorSize);

//        if (pcl_isnan(descriptors->points[i].descriptor[0])){
//            descriptors->points[i].descriptor[0] = 0;
//        }
        for (size_t j = 0; j < DescriptorSize; ++j){

            if (pcl_isnan(descriptors->points[i].descriptor[j])){
                descriptors->points[i].descriptor[j] = 0;
            }

//            data_tmp[j] = descriptors->points[i].descriptor[j];
        }
//        //normalize descriptor
//        std::vector<float> value_descriptor_scaled = minMaxScaler(data_tmp);

////        float sum = 0;
//        for (size_t j = 0; j < DescriptorSize; ++j){
//            descriptors->points[i].descriptor[j] = value_descriptor_scaled.at(j);
////            sum += descriptors->points[i].descriptor[j];
//        }
////        cout<<"GSHOTPyramid::sum of the descriptor normalized : "<< sum << endl;

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

const vector<vector<GSHOTPyramid::Level> > & GSHOTPyramid::levels() const{
    
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

