#ifndef TENSOR3D_H
#define TENSOR3D_H
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
#include "typedefs.h"

//Subsection = interval ?
//Padding ?
#ifdef _OPENMP
#include <omp.h>
#endif
#include <omp.h>


typedef float Scalar;

template <typename Type>
class Tensor3D{
public:
    Tensor3D() : tensor( Eigen::Tensor< Type, 3, Eigen::RowMajor>(0,0,0))
    {}


    Tensor3D( Eigen::TensorMap< Eigen::Tensor< Type, 3, Eigen::RowMajor> > t)
        : tensor( t)
    {}

    Tensor3D( Eigen::Tensor< Type, 3, Eigen::RowMajor>& t)
        : tensor( t)
    {}

    Tensor3D( int s1, int s2, int s3)
    {
        tensor = Eigen::Tensor< Type, 3, Eigen::RowMajor>(s1, s2, s3);
    }

    Eigen::Tensor< Type, 3, Eigen::RowMajor>& operator()(){
        return tensor;
    }

    const Eigen::Tensor< Type, 3, Eigen::RowMajor>& operator()() const{
        return tensor;
    }

    int rows() const{
        return tensor.dimension(1);
    }
    int cols() const {
        return tensor.dimension(2);
    }
    int depths() const{
        return tensor.dimension(0);
    }
    int size() const{
        return tensor.size();//rows() * cols() * depths();
    }

    bool isZero() const{
        for (int i = 0; i < depths(); ++i) {
            for (int j = 0; j < rows(); ++j) {
                for (int k = 0; k < cols(); ++k) {
                    if ( tensor(i, j, k) != 0){
                        return false;
                    }
                }
            }
        }
        return true;
    }

    //Level
    Tensor3D<Scalar> convolve( Tensor3D< Type> filter) const{
        Tensor3D<Scalar> res( depths() - filter.depths() + 1,
                  rows() - filter.rows() + 1,
                  cols() - filter.cols() + 1);

        res().setConstant( 0);

#pragma omp parallel for num_threads(omp_get_max_threads())
        for (int z = 0; z < res.depths(); ++z) {
            for (int y = 0; y < res.rows(); ++y) {
                for (int x = 0; x < res.cols(); ++x) {

                    for (int dz = 0; dz < filter.depths(); ++dz) {
                        for (int dy = 0; dy < filter.rows(); ++dy) {
                            for (int dx = 0; dx < filter.cols(); ++dx) {

                                res()(z, y, x) += tensor(z+dz, y+dy, x+dx).matrix().dot(filter()(dz, dy, dx).matrix());

                            }
                        }
                    }

                }
            }
        }
        return res;
    }

    //Level
    Tensor3D< Type> agglomerate() const{
        Tensor3D< Type> res(1,1,1);
        res().setConstant( Type::Zero());
#pragma omp parallel for num_threads(omp_get_max_threads())
        for (int z = 0; z < depths(); ++z) {
            for (int y = 0; y < rows(); ++y) {
                for (int x = 0; x < cols(); ++x) {
                    res()(0,0,0) += tensor(z, y, x);
                }
            }
        }
//        t()(0,0,0) = t()(0,0,0) / (cols() * rows() * depths());
        return res;
    }

    //Tensor3DF
    Tensor3D< Type> agglomerate( int size) const{
        Tensor3D< Type> t(1,1,size);
        t().setConstant( 0);
        for (int z = 0; z < depths(); ++z) {
            for (int y = 0; y < rows(); ++y) {
#pragma omp parallel for num_threads(omp_get_max_threads())
                for (int x = 0; x < cols(); ++x) {
                    t()(0,0,x%size) += tensor(z, y, x);
                }
            }
        }
//        t()(0,0,0) = t()(0,0,0) / (cols() * rows() * depths());
        return t;
    }


    //TODO replace by TensorMap
    //return a block of size (p, q, r) from point (z, y, x)
    Tensor3D< Type> block(int z, int y, int x, int p, int q, int r){
        Tensor3D< Type> t(p, q, r);
#pragma omp parallel for
        for (int i = 0; i < p; ++i) {
#pragma omp parallel for
            for (int j = 0; j < q; ++j) {
#pragma omp parallel for
                for (int k = 0; k < r; ++k) {
                    t()(i,j,k) = tensor(z+i, y+j, x+k);
                }
            }
        }
        return t;
//        Eigen::array<int, 3> offsets = {z, y, x};
//        Eigen::array<int, 3> extents = {p,q,r};
////        Eigen::array<int, 3> three_dims{{rootSize().first, rootSize().second, rootSize().third}};

//        return Tensor3D< Type>(tensor.slice(offsets, extents)/*.reshape(three_dims)*/);
    }

    //return a block of size (p, q, r) from point (z, y, x)
    const Tensor3D< Type> block(int z, int y, int x, int p, int q, int r) const{
        Tensor3D< Type> t(p, q, r);
#pragma omp parallel for
        for (int i = 0; i < p; ++i) {
#pragma omp parallel for
            for (int j = 0; j < q; ++j) {
#pragma omp parallel for
                for (int k = 0; k < r; ++k) {
                    t()(i,j,k) = tensor(z+i, y+j, x+k);
                }
            }
        }
        return t;
//        Eigen::array<int, 3> offsets = {z, y, x};
//        Eigen::array<int, 3> extents = {p,q,r};
////        Eigen::array<int, 3> three_dims{{rootSize().first, rootSize().second, rootSize().third}};

//        return Tensor3D< Type>(tensor.slice(offsets, extents)/*.reshape(three_dims)*/);
    }



//    Eigen::Map< Eigen::Matrix<Type, 1, Eigen::Dynamic, Eigen::RowMajor> > block(int z, int y, int x, int p, int q, int r){
////        Type* pointer = tensor.data() ;/*+ x + y * cols() + z * rows() * cols();*/
////        return Eigen::TensorMap< Eigen::Tensor<Type, 3, Eigen::RowMajor> >(pointer, p, q, r);
//        Type* pointer = tensor.data();

//    }

    //row d'une matrice --> renvoie ligne de la matrice
    Eigen::Matrix<Type, 1, Eigen::Dynamic, Eigen::RowMajor> row( int z, int y) const{
        Eigen::Matrix<Type, 1, Eigen::Dynamic, Eigen::RowMajor> res(tensor.dimension(2));
        for (int x = 0; x < tensor.dimension(2); ++x) {
            res( 0, x) = tensor(z, y, x);
        }
        return res;
    }

    Eigen::Matrix<Type, 1, Eigen::Dynamic, Eigen::RowMajor> col( int z, int x) const{
        Eigen::Matrix<Type, 1, Eigen::Dynamic, Eigen::RowMajor> res(tensor.dimension(1));
        for (int y = 0; y < tensor.dimension(1); ++y) {
            res( 0, y) = tensor(z, y, x);
        }
        return res;
    }

    Eigen::Matrix<Type, 1, Eigen::Dynamic, Eigen::RowMajor> depth( int y, int x) const{
        Eigen::Matrix<Type, 1, Eigen::Dynamic, Eigen::RowMajor> res(tensor.dimension(0));
        for (int z = 0; z < tensor.dimension(0); ++z) {
            res( 0, z) = tensor(z, y, x);
        }
        return res;
    }

    //TODO
    Type sum() const{
                    Type res = 0;
                    for (int i = 0; i < depths(); ++i) {
                        for (int j = 0; j < rows(); ++j) {
                            for (int k = 0; k < cols(); ++k) {
                                res += tensor(i, j, k);
                            }
                        }
                    }
        return res;//((Eigen::Tensor< Type, 3, Eigen::RowMajor>)tensor.sum())(0);
    }

    Type squaredNorm() const{
                    Type res = 0;
                    for (int i = 0; i < depths(); ++i) {
                        for (int j = 0; j < rows(); ++j) {
                            for (int k = 0; k < cols(); ++k) {
                                res += tensor(i, j, k) * tensor(i, j, k);
                            }
                        }
                    }
        return res;//((Eigen::Tensor< Type, 3, Eigen::RowMajor>)tensor.sum())(0);
    }


    Eigen::Tensor<Type, 3, Eigen::RowMajor> tensor;
};

/// Type of a pyramid level (matrix of cells).
typedef Tensor3D<Scalar> Tensor3DF;
typedef Tensor3D<int> Tensor3DI;

#endif // TENSOR3D_H
