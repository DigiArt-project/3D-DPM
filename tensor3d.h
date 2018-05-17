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
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <vector>

//Other
#include "typedefs.h"
#include <boost/filesystem.hpp>


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

    void setZero(){
//#pragma omp parallel for
        for (int z = 0; z < depths(); ++z) {
            for (int y = 0; y < rows(); ++y) {
                for (int x = 0; x < cols(); ++x) {
                    *(tensor(z, y, x))=0;
                }
            }
        }
    }

    //Level
    bool hasNegValue() const{
//#pragma omp parallel for
        for (int z = 0; z < depths(); ++z) {
            for (int y = 0; y < rows(); ++y) {
                for (int x = 0; x < cols(); ++x) {
                    for (int j = 0; j < 352; ++j) {
                        if( tensor(z, y, x)(j) < 0) return true;
                    }
                }
            }
        }
        return false;
    }

    //Level
    Tensor3D< Type> agglomerateBlock(int z, int y, int x, int p, int q, int r) const{
        if(z+p>depths() || y+q>rows() || x+r>cols() || z < 0 || y < 0 || x < 0){
            cerr<<"agglomerateBlock:: Try to access : "<<z+p<<" / "<<y+q<<" / "<<x+r<<" on matrix size : "
               <<depths()<<" / "<<rows()<<" / "<<cols()<<endl;
//            exit(0);
            p = min(z+p, depths()) - z;
            q = min(y+q, rows()) - y;
            r = min(x+r, cols()) - x;
        }
        Tensor3D< Type> res(1,1,1);
        res().setConstant( Type::Zero());
//#pragma omp parallel for num_threads(omp_get_max_threads())
        for (int i = z; i < p; ++i) {
            for (int j = y; j < q; ++j) {
                for (int k = x; k < r; ++k) {
                    res()(0,0,0) += tensor(i, j, k);
                }
            }
        }
    //        t()(0,0,0) = t()(0,0,0) / (cols() * rows() * depths());
        return res;
    }


    //Level
    Tensor3D<Scalar> convolve( Tensor3D< Type> filter) const{
        cout<<"tensor3D::convolve ..."<<endl;

        Tensor3D<Scalar> res( depths() - filter.depths() + 1,
                              rows() - filter.rows() + 1,
                              cols() - filter.cols() + 1);

        res().setConstant( 0);


//        Type filterMean = filter.sum() / Scalar(filter.size());

//        for(int i=0; i<2; ++i){
//            cout<<"tensor3D::convolve filterMean("<<i<<") = "<<filterMean(i)<<endl;
//        }

//#pragma omp parallel for num_threads(omp_get_max_threads())
        for (int z = 0; z < res.depths(); ++z) {
            for (int y = 0; y < res.rows(); ++y) {
                for (int x = 0; x < res.cols(); ++x) {

//                    Type tensorMean = agglomerateBlock(z, y, x, filter.depths(), filter.rows(), filter.cols())()(0,0,0) /
//                            Scalar(filter.size());


//                    Type squaredNormTensor;
//                    Type squaredNormFilter;
//                    squaredNormTensor.setConstant( 0);
//                    squaredNormFilter.setConstant( 0);
//                    Scalar aux( 0);

                    for (int dz = 0; dz < filter.depths(); ++dz) {
                        for (int dy = 0; dy < filter.rows(); ++dy) {
                            for (int dx = 0; dx < filter.cols(); ++dx) {

                                Type normTensor = tensor(z+dz, y+dy, x+dx) /*- tensorMean*/;
                                Type normFilter = filter()(dz, dy, dx) /*- filterMean*/;

//                                Type sum = filter.sum();
//                                for(int i=0; i<352; ++i){
//                                    if (sum(i) !=0) normFi/*- tensorMean*/lter(i) /= sum(i);
//                                    else normFilter(i) = 0;
//                                }
//                                squaredNormTensor += normTensor * normTensor;
//                                squaredNormFilter += normFilter * normFilter;
//                                aux += normTensor.matrix().dot(normFilter.matrix()) * normTensor.matrix().dot(normFilter.matrix());
                                res()(z, y, x) += normTensor.matrix().dot(normFilter.matrix());

//                                    cout<<"tensor3D::convolve res()(z, y, x) = "<<res()(z, y, x)<<endl;
//                                    cout<<"tensor3D::convolve normTensor = "<<normTensor<<endl;

//                                cout<<"tensor3D::conv tensor(z+dz, y+dy, x+dx).matrix() : "<<endl
//                                   <<tensor(z+dz, y+dy, x+dx).matrix().transpose()<<endl;
//                                cout<<"tensor3D::conv filter()(dz, dy, dx).matrix() : "<<endl
//                                   <<filter()(dz, dy, dx).matrix().transpose()<<endl;
//                                cout<<"tensor3D::conv dot product : "<<endl
//                                   <<tensor(z+dz, y+dy, x+dx).matrix().dot(filter()(dz, dy, dx).matrix())<<endl;
                            }
                        }
                    }

//                    res()(z, y, x) /= /*sqrt*/(squaredNormTensor.matrix().sum() * squaredNormFilter.matrix().sum());
//                    cout<<"tensor3D::convolve squaredNormTensor.matrix().sum() = "<<squaredNormTensor.matrix().sum()<<endl;
//                    cout<<"tensor3D::convolve squaredNormFilter.matrix().sum() = "<<squaredNormFilter.matrix().sum()<<endl;

//                    cout<<"tensor3D::convolve res()(z, y, x) = "<<squaredNormTensor.matrix().dot(squaredNormFilter.matrix())<<endl;


                }
            }
        }
        return res;
    }




    //Level
    Tensor3D< Type> agglomerate() const{
        Tensor3D< Type> res(1,1,1);
        res().setConstant( Type::Zero());
//#pragma omp parallel for num_threads(omp_get_max_threads())
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
//#pragma omp parallel for num_threads(omp_get_max_threads())
                for (int x = 0; x < cols(); ++x) {
                    t()(0,0,x%size) += tensor(z, y, x);
                }
            }
        }
//        t()(0,0,0) = t()(0,0,0) / (cols() * rows() * depths());
        return t;
    }

    //return a block of size (p, q, r) from point (z, y, x)
    Tensor3D< Scalar*> blockLink(int z, int y, int x, int p, int q, int r){
        if(z+p>depths() || y+q>rows() || x+r>cols() || z < 0 || y < 0 || x < 0){
            cerr<<"blockLink:: Try to access : "<<z+p<<" / "<<y+q<<" / "<<x+r<<" on matrix size : "
               <<depths()<<" / "<<rows()<<" / "<<cols()<<endl;
            exit(0);
        }
        Tensor3D< Scalar*> res(p, q, r);
//#pragma omp parallel for
        for (int i = 0; i < p; ++i) {
            for (int j = 0; j < q; ++j) {
                for (int k = 0; k < r; ++k) {
                    res()(i,j,k) = &tensor(z+i, y+j, x+k);
                }
            }
        }
        return res;
    }



    //TODO replace by TensorMap
    //return a block of size (p, q, r) from point (z, y, x)
    Tensor3D< Type> block(int z, int y, int x, int p, int q, int r) const{
        if(z+p>depths() || y+q>rows() || x+r>cols() || z < 0 || y < 0 || x < 0){
            cerr<<"block:: Try to access : "<<z+p<<" / "<<y+q<<" / "<<x+r<<" on matrix size : "
               <<depths()<<" / "<<rows()<<" / "<<cols()<<endl;
            exit(0);
        }
        Tensor3D< Type> t(p, q, r);
//#pragma omp parallel for
        for (int i = 0; i < p; ++i) {
            for (int j = 0; j < q; ++j) {
                for (int k = 0; k < r; ++k) {
                    t()(i,j,k) = tensor(z+i, y+j, x+k);
                }
            }
        }
        return t;
    }

//    //return a block of size (p, q, r) from point (z, y, x)
//    const Tensor3D< Type> block(int z, int y, int x, int p, int q, int r) const{
//        Tensor3D< Type> t(p, q, r);
//#pragma omp parallel for
//        for (int i = 0; i < p; ++i) {
//            for (int j = 0; j < q; ++j) {
//                for (int k = 0; k < r; ++k) {
//                    t()(i,j,k) = tensor(z+i, y+j, x+k);
//                }
//            }
//        }
//        return t;
//    }



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
                    Type res(0);
                    for (int i = 0; i < depths(); ++i) {
                        for (int j = 0; j < rows(); ++j) {
                            for (int k = 0; k < cols(); ++k) {
                                res += tensor(i, j, k);
                            }
                        }
                    }
        return res;
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
        return res;
    }


    Eigen::Tensor<Type, 3, Eigen::RowMajor> tensor;
};

/// Type of a pyramid level (matrix of cells).
typedef Tensor3D<Scalar> Tensor3DF;
typedef Tensor3D<int> Tensor3DI;

#endif // TENSOR3D_H
