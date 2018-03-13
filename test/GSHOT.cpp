//EIGEN
#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/common/common_headers.h>
#include <vector>

//Other
//#include "typedefs.h"

//Subsection = interval ?
//Padding ?
#ifdef _OPENMP
#include <omp.h>
#endif


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;


/* Define some custom types to make the rest of our code easier to read */
typedef pcl::ReferenceFrame RFType;

// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudT;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudConstPtr;
//Vector of point cloud
typedef std::vector<PointCloudPtr> cloudVectorT;

// Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> SurfaceNormals;
typedef pcl::PointCloud<NormalType>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalType>::ConstPtr SurfaceNormalsConstPtr;

// Define "Descriptors" to be a pcl::PointCloud of SHOT points. Can change
//typedef pcl::SHOT352 LocalDescriptor;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> Descriptors;
typedef pcl::PointCloud<DescriptorType>::Ptr DescriptorsPtr;
typedef pcl::PointCloud<DescriptorType>::ConstPtr DescriptorsConstPtr;



static const int DescriptorSize = 1;
typedef float Scalar;


/// Type of a matrix.
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;

template <typename Type>
struct Tensor : Eigen::Tensor<Type, 3, Eigen::RowMajor>{
    Tensor( int s1, int s2, int s3) : Eigen::Tensor<Type, 3, Eigen::RowMajor>(s1, s2, s3){}
    //row d'une matrice --> renvoie ligne de la matrice

    Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    row( int i) const{
        Eigen::Matrix<Type, this->dimension(0), this->dimension(1), Eigen::RowMajor> res;
        return res.setZero();
    }

    int rows(){
        return *this->dimension(0);
    }
    int cols(){
        return *this->dimension(1);
    }
    int depths(){
        return *this->dimension(2);
    }

};

/// Type of a pyramid level cell (fixed-size array of length NbFeatures).
typedef Eigen::Array<Scalar, DescriptorSize, 1> Cell;

/// Type of a pyramid level (matrix of cells).
//typedef Tensor<Cell> Level;

typedef Matrix Level;




int main(){

    Level x(5,5);
    x << 1,2,3,4,5,
            6,7,8,9,10,
            11,12,13,14,15,
            16,17,18,19,20,
            21,22,23,24,25;
    Level y(3,3);
    y <<1,2,3,
            4,5,6,
            7,8,9;
    // Nothing to do if x is smaller than y
    if ((x.rows() < y.rows()) || (x.cols() < y.cols())) {
        Matrix z = Matrix();
        return 0;
    }

    Matrix z = Matrix::Zero( x.rows() - y.rows() + 1, x.cols() - y.cols() + 1);

    std::cout<<"x = "<< x<<std::endl;
    std::cout<<"y = "<< y<<std::endl;
    std::cout<<"z = "<< z<<std::endl;

    for (int i = 0; i < z.rows(); ++i) {
        for (int j = 0; j < y.rows(); ++j) {
            const Eigen::Map<const Matrix> mapx(
                        reinterpret_cast<const Scalar *>(x.row(i + j).data()), z.cols(), y.cols() * DescriptorSize);

            std::cout<<"x.row(i + j) = " << x.row(i + j)<<std::endl;
            const Eigen::Map<const Eigen::RowVectorXf> mapy(
                        reinterpret_cast<const Scalar *>(y.row(j).data()), y.cols() * DescriptorSize);
            std::cout<<"for i = " << i<< ", et j = "<< j<<std::endl;
            std::cout<<"mapx = " << mapx<<std::endl;
            std::cout<<"mapy = " << mapy<<std::endl;

            z.row(i).noalias() += mapy * mapx.transpose();
            std::cout<<"z.row(i) = "<< z.row(i)<<std::endl;

        }
    }
    return 0;
}
