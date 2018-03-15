#include "GSHOTPyramid.h"

#include <cstdlib>
#include <sys/timeb.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;

template <typename Type>
std::ostream& operator<<(std::ostream& os, GSHOTPyramid::Tensor3D<Type> t){
    for (int i = 0; i < t().dimension(2); ++i) {
        for (int j = 0; j < t().dimension(1); ++j) {
            for (int k = 0; k < t().dimension(0); ++k) {
                os << t()(i,j,k) << " ";
            }
            os << "\n";
        }
        os << "\n";
    }
    
    return os;
 }

class Test{
public:
    Test(){
    }

    //Create tensor et parcour valeur par valeur avec pointeur
    void tensorParcours(){
        int size = 3;
        GSHOTPyramid::Tensor3D<int> level(size,size, size);
        level().setZero();
        int p = 0;
        //Row colonne depth
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                for (int k = 0; k < size; ++k) {
                    level()(i,j,k) = p;
                    ++p;

                }
            }
        }


        std::cout << level() << std::endl;
        //Parcour de la dernière dimension à la premiere dimension
        //(depth, row, col)
        int* jony = &(level()(0));
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                for (int k = 0; k < size; ++k) {
                    std::cout << *jony++  << " " << std::endl;
                }
            }
        }
    }

    //Tester convolution
    void convolve(){
        int descriptorSize = 3;
        GSHOTPyramid::Level x(5,5,5), y(3,3,3);
        GSHOTPyramid::Cell cell1(descriptorSize), cell2(descriptorSize);
        cell1 << 1,2,3;
        cell2 << 3,1,3;
        x().setConstant(cell1);
//        x(0,0,0) = 2;
        y().setConstant(cell2);
//        y <<1,2,3,
//                4,5,6,
//                7,8,9;


        Eigen::Tensor<float, 3, RowMajor> dx(x().dimension(0), x().dimension(1), x().dimension(2) * descriptorSize),
                                          dy(y().dimension(0), y().dimension(1), y().dimension(2) * descriptorSize);
        Eigen::Tensor<float, 3, RowMajor> res( dx.dimension(0) - dy.dimension(0) + 1,
                                       dx.dimension(1) - dy.dimension(1) + 1,
                                       dx.dimension(2) - dy.dimension(2) + 1);
        GSHOTPyramid::Tensor3D<float> resPerso( dx.dimension(0) - dy.dimension(0) + 1,
                                       dx.dimension(1) - dy.dimension(1) + 1,
                                       dx.dimension(2) - dy.dimension(2) + 1);

        for (int i = 0; i < x().dimension(0); ++i) {
            for (int j = 0; j < x().dimension(1); ++j) {
                for (int k = 0; k < x().dimension(2); ++k) {
                    for (int l = 0; l < descriptorSize; ++l) {
                        dx(i,j, k * descriptorSize + l) = x()(i,j,k).coeff(l);
                    }
                }
            }
        }

        for (int i = 0; i < y().dimension(0); ++i) {
            for (int j = 0; j < y().dimension(1); ++j) {
                for (int k = 0; k < y().dimension(2); ++k) {
                    for (int l = 0; l < descriptorSize; ++l) {
                        dy(i,j, k * descriptorSize + l) = y()(i,j,k).coeff(l);
                    }
                }
            }
        }

        Eigen::array<ptrdiff_t, 3> dims({0, 1, 2});
        res = dx.convolve(dy, dims);
        resPerso() = dx.convolve(dy, dims);

        std::cout << "dx = \n" << dx << std::endl;
        std::cout << "dy = \n" << dy << std::endl;
        std::cout << "res = \n" << res << std::endl;
        std::cout << "resPerso = \n" << resPerso() << std::endl;


        assert(true);
    }
};

int main(){

    
    Test t;
    //t.tensorParcours();
    t.convolve();

   

    
    return 0;
}

