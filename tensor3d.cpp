#include "tensor3d.h"

//Tensor3D::Tensor3D() : tensor( Eigen::Tensor<typename Type, 3, Eigen::RowMajor>(0,0,0))
//{}

//Tensor3D::Tensor3D( int s1, int s2, int s3)
//{
//    tensor = Eigen::Tensor<typename Type, 3, Eigen::RowMajor>(s1, s2, s3);
//}

////row d'une matrice --> renvoie ligne de la matrice
//Eigen::Matrix<typename Type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Tensor3D::row( int i) const{
//    Eigen::Matrix<typename Type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> res(tensor.dimension(0),tensor.dimension(1));
//    return res.setZero();
//}

//Eigen::Tensor<typename Type, 3, Eigen::RowMajor>& Tensor3D::operator()(){
//    return tensor;
//}

//const Eigen::Tensor<typename Type, 3, Eigen::RowMajor>& Tensor3D::operator()() const{
//    return tensor;
//}

//int Tensor3D::rows() const{
//    return tensor.dimension(0);
//}
//int Tensor3D::cols() const {
//    return tensor.dimension(1);
//}
//int Tensor3D::depths() const{
//    return tensor.dimension(2);
//}
//int Tensor3D::size() const{
//    return tensor.size();//rows() * cols() * depths();
//}

////TODO replace by TensorMap
////return a block of size (p, q, r) from point (z, y, x)
//Tensor3D<typename Type> Tensor3D::block(int z, int y, int x, int p, int q, int r){
//    Tensor3D<typename Type> t(p, q, r);
//    for (int i = 0; i < p; ++i) {
//        for (int j = 0; j < q; ++j) {
//            for (int k = 0; k < r; ++k) {
//                t()(i,j,k) = tensor(z+i, y+j, x+k);
//            }
//        }
//    }
//    return t;
//}

////return a block of size (p, q, r) from point (z, y, x)
//const Tensor3D<typename Type> Tensor3D::block(int z, int y, int x, int p, int q, int r) const{
//    Tensor3D<typename Type> t(p, q, r);
//    for (int i = 0; i < p; ++i) {
//        for (int j = 0; j < q; ++j) {
//            for (int k = 0; k < r; ++k) {
//                t()(i,j,k) = tensor(z+i, y+j, x+k);
//            }
//        }
//    }
//    return t;
//}

//typename Type Tensor3D::sum() const{
////                double res = 0;
////                for (int i = 0; i < depths(); ++i) {
////                    for (int j = 0; j < rows(); ++j) {
////                        for (int k = 0; k < cols(); ++k) {
////                            res += tensor(i, j, k);
////                        }
////                    }
////                }
//    return ((Eigen::Tensor<typename Type, 3, Eigen::RowMajor>)tensor.sum())(0);
//}

