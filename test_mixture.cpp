
#include "Mixture.h"

#include <cstdlib>
#include <sys/timeb.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;

    class Test{
    public:
        Test(){
        }

    };

int main(){


    Test t;
    const std::vector<Model>& models = { Model( Model::triple<int, int, int>(5,5,5))};
    int nbComponents = 1;
//    const std::vector<Scene>& scenes = {Scene()};
//    Object::Name name = Object::AEROPLANE;


//    Mixture mixture( models);




    return 0;
}

