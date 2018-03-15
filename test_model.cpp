#include "Model.h"

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

    }
};

int main(){

    
    Test t;
    t.tensorParcours();

   

    
    return 0;
}

