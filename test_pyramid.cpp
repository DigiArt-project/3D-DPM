#include "GSHOTPyramid.h"

#include <cstdlib>
#include <sys/timeb.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;

template <typename Type>
std::ostream& operator<<(std::ostream& os, GSHOTPyramid::Tensor<Type> t){
    for (int i = 0; i < t.dimension(2); ++i) {
        for (int j = 0; j < t.dimension(1); ++j) {
            for (int k = 0; k < t.dimension(0); ++k) {
                os << t(i,j,k) << " ";
            }
            os << "\n";
        }
        os << "\n";
    }
    
    return os;
 }

int main(){
    //Create tensor et parcour valeur par valeur avec pointeur
    //Tester convolution
    
    int size = 3;
    GSHOTPyramid::Tensor<int> level(size,size, size);
    level.setZero();
    int p = 0;
    //Row colonne depth
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            for (int k = 0; k < size; ++k) {
                level(i,j,k) = p;
                ++p;
                
            }
        }
    }
    
    
    std::cout << level << std::endl;
    //Parcour de la dernière dimension à la premiere dimension
    //(depth, row, col)
    int* jony = &(level(0));
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            for (int k = 0; k < size; ++k) {
                std::cout << *jony++  << " " << std::endl;
            }
        }
    }

   

    
    return 0;
}

