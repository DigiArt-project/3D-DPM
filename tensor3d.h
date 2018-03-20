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


typedef float Scalar;

template <typename Type>
class Tensor3D{
public:
    Tensor3D() : tensor( Eigen::Tensor< Type, 3, Eigen::RowMajor>(0,0,0))
    {}


    Tensor3D( Eigen::TensorMap< Eigen::Tensor< Type, 3, Eigen::RowMajor> > t)
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

    //TODO replace by TensorMap
    //return a block of size (p, q, r) from point (z, y, x)
    Tensor3D< Type> block(int z, int y, int x, int p, int q, int r){
        Tensor3D< Type> t(p, q, r);
        for (int i = 0; i < p; ++i) {
            for (int j = 0; j < q; ++j) {
                for (int k = 0; k < r; ++k) {
                    t()(i,j,k) = tensor(z+i, y+j, x+k);
                }
            }
        }
        return t;
    }

    //return a block of size (p, q, r) from point (z, y, x)
    const Tensor3D< Type> block(int z, int y, int x, int p, int q, int r) const{
        Tensor3D< Type> t(p, q, r);
        for (int i = 0; i < p; ++i) {
            for (int j = 0; j < q; ++j) {
                for (int k = 0; k < r; ++k) {
                    t()(i,j,k) = tensor(z+i, y+j, x+k);
                }
            }
        }
        return t;
    }

    //row d'une matrice --> renvoie ligne de la matrice
    Eigen::Matrix<Type, 1, Eigen::Dynamic, Eigen::RowMajor> row( int z, int y) const{
        Eigen::Matrix<Type, 1, Eigen::Dynamic, Eigen::RowMajor> res(tensor.dimension(2));
        for (int x = 0; x < tensor.dimension(2); ++x) {
            res( 0, x) = tensor(z, y, x);
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


    Eigen::Tensor<Type, 3, Eigen::RowMajor> tensor;
};

/// Type of a pyramid level (matrix of cells).
typedef Tensor3D<Scalar> Tensor3DF;

#endif // TENSOR3D_H
