#include "Model.h"

#include <cstdlib>
#include <sys/timeb.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;

    class Test{
    public:
        Test(){
        }

        void initializeParts(int nbParts, Model::triple<int, int, int> partSize)
        {
            GSHOTPyramid::Cell nullCell;
            nullCell.setZero( GSHOTPyramid::DescriptorSize);
            // The model stay unmodified if any of the parameter is invalid
            if ((nbParts <= 0) || (partSize.first <= 0) || (partSize.second <= 0) || (partSize.third <= 0)) {
                cerr << "Attempting to initialize parts in an empty model" << endl;
                return;
            }

            // Upsample the root filter by a factor 2 using bicubic interpolation
            const GSHOTPyramid::Level & root = parts_[0].filter;


            GSHOTPyramid::Level root2x = GSHOTPyramid::Level(2 * root.depths(), 2 * root.rows(), 2 * root.cols());
            root2x().setConstant( nullCell);

            //TODO
            // Bicubic interpolation matrix for x = y = 0.25
            const double bicubic[4][4] =
            {
                { 0.004943847656,-0.060974121094,-0.015930175781, 0.001647949219},
                {-0.060974121094, 0.752014160156, 0.196472167969,-0.020324707031},
                {-0.015930175781, 0.196472167969, 0.051330566406,-0.005310058594},
                { 0.001647949219,-0.020324707031,-0.005310058594, 0.000549316406}
            };

            for (int z = 0; z < root.depths(); ++z) {
                for (int y = 0; y < root.rows(); ++y) {
                    for (int x = 0; x < root.cols(); ++x) {
                        for (int i = 0; i < 4; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                const int z2 = min(max(z + i - 2, 0), static_cast<int>(root.depths()) - 1);
                                const int z1 = min(max(z + i - 1, 0), static_cast<int>(root.depths()) - 1);
                                const int y2 = min(max(y + i - 2, 0), static_cast<int>(root.rows()) - 1);
                                const int y1 = min(max(y + i - 1, 0), static_cast<int>(root.rows()) - 1);
                                const int x2 = min(max(x + j - 2, 0), static_cast<int>(root.cols()) - 1);
                                const int x1 = min(max(x + j - 1, 0), static_cast<int>(root.cols()) - 1);
                            //TODO
        //                        root2x()(y * 2    , x * 2    ) += bicubic[3 - i][3 - j] * root()(y2, x2);
        //                        root2x()(y * 2    , x * 2 + 1) += bicubic[3 - i][    j] * root()(y2, x1);
        //                        root2x()(y * 2 + 1, x * 2    ) += bicubic[    i][3 - j] * root()(y1, x2);
        //                        root2x()(y * 2 + 1, x * 2 + 1) += bicubic[    i][    j] * root()(y1, x1);
                            }
                        }
                    }
                }
            }

            // Compute the energy of each cell
            Tensor3D<GSHOTPyramid::Scalar> energy(root2x.depths(), root2x.rows(), root2x.cols());

            for (int z = 0; z < root2x.depths(); ++z) {
                for (int y = 0; y < root2x.rows(); ++y) {
                    for (int x = 0; x < root2x.cols(); ++x) {
                        root2x()(z, y, x).cwiseMax(0);

                        energy()(z, y, x) = 0;

                        for (int i = 0; i < GSHOTPyramid::DescriptorSize; ++i)
                            energy()(z, y, x) += root2x()(z, y, x)(i) * root2x()(z, y, x)(i);
                    }
                }
            }

            // Assign each part greedily to the region of maximum energy
            parts_.resize(nbParts + 1);

            for (int i = 0; i < nbParts; ++i) {
                double maxEnergy = 0.0;
                int argX = 0;
                int argY = 0;
                int argZ = 0;

                for (int z = 0; z <= energy.depths() - partSize.first; ++z) {
                    for (int y = 0; y <= energy.rows() - partSize.second; ++y) {
                        for (int x = 0; x <= energy.cols() - partSize.third; ++x) {
                            const double e = energy.block(z, y, x, partSize.first, partSize.second, partSize.third).sum();

                            if (e > maxEnergy) {
                                maxEnergy = e;
                                argX = x;
                                argY = y;
                                argZ = z;
                            }
                        }
                    }
                }

//                // Initialize the part
//                parts_[i + 1].filter = root2x.block(argZ, argY, argX, partSize.first, partSize.second, partSize.third);
//                parts_[i + 1].offset(0) = argZ;
//                parts_[i + 1].offset(1) = argY;
//                parts_[i + 1].offset(2) = argX;

//                // Set the energy of the part to zero
//                energy.block(argZ, argY, argX, partSize.first, partSize.second, partSize.third)().setZero();
//            }

//            // Retry 10 times from randomized starting points
//            double bestCover = 0.0;
//            vector<Model::Part> best(parts_); // The best so far is the current one

//            for (int i = 0; i < 10; ++i) {
//                vector<Model::Part> tmp(parts_); // Try from the current one

//                // Remove a part at random and look for the best place to put it
//                for (int j = 0; j < 100 * nbParts; ++j) {
//                    // Recompute the energy
//                    for (int z = 0; z < root2x.depths(); ++z) {
//                        for (int y = 0; y < root2x.rows(); ++y) {
//                            for (int x = 0; x < root2x.cols(); ++x) {
//                                energy()(z, y, x) = 0;

//                                for (int i = 0; i < GSHOTPyramid::DescriptorSize; ++i)
//                                    energy()(z, y, x) += root2x()(z, y, x)(i) * root2x()(z, y, x)(i);
//                            }
//                        }
//                    }

//                    // Select a part at random
//                    const int part = rand() % nbParts;

//                    // Zero out the energy covered by the other parts
//                    for (int k = 0; k < nbParts; ++k)
//                        if (k != part)
//                            energy.block(tmp[k + 1].offset(0), tmp[k + 1].offset(1), tmp[k + 1].offset(2), partSize.first,
//                                         partSize.second, partSize.third)().setZero();

//                    // Find the region of maximum energy
//                    double maxEnergy = 0.0;
//                    int argX = 0;
//                    int argY = 0;
//                    int argZ = 0;

//                    for (int z = 0; z <= energy.depths() - partSize.first; ++z) {
//                        for (int y = 0; y <= energy.rows() - partSize.second; ++y) {
//                            for (int x = 0; x <= energy.cols() - partSize.third; ++x) {
//                                const double e = ((Eigen::Tensor<float, 3, Eigen::RowMajor>) energy.block(z, y, x,
//                                                                                                          partSize.first,
//                                                                                                          partSize.second,
//                                                                                                          partSize.third)().sum())(0);

//                                if (e > maxEnergy) {
//                                    maxEnergy = e;
//                                    argX = x;
//                                    argY = y;
//                                    argZ = z;
//                                }
//                            }
//                        }
//                    }

//                    // Initialize the part
//                    tmp[part + 1].filter = root2x.block(argZ, argY, argX, partSize.first, partSize.second, partSize.third);
//                    tmp[part + 1].offset(0) = argZ;
//                    tmp[part + 1].offset(1) = argY;
//                    tmp[part + 1].offset(2) = argX;
//                }

//                // Compute the energy covered by this part arrangement
//                double cover = 0.0;

//                // Recompute the energy
//                for (int z = 0; z < root2x.depths(); ++z) {
//                    for (int y = 0; y < root2x.rows(); ++y) {
//                        for (int x = 0; x < root2x.cols(); ++x) {
//                            energy()(z, y, x) = 0;

//                            for (int i = 0; i < GSHOTPyramid::DescriptorSize; ++i)
//                                energy()(z, y, x) += root2x()(z, y, x)(i) * root2x()(z, y, x)(i);
//                        }
//                    }
//                }

//                for (int j = 0; j < nbParts; ++j) {
//                    // Add the energy of the part
//                    cover += ((Eigen::Tensor<float, 3, Eigen::RowMajor>) energy.block(tmp[j + 1].offset(0), tmp[j + 1].offset(1), tmp[j + 1].offset(2), partSize.first,
//                                          partSize.second, partSize.third)().sum())(0);

//                    // Set the energy of the part to zero
//                    energy.block(tmp[j + 1].offset(0), tmp[j + 1].offset(1), tmp[j + 1].offset(2), partSize.first,
//                                 partSize.second, partSize.third)().setZero();
//                }

//                if (cover > bestCover) {
//                    bestCover = cover;
//                    best = tmp;
//                }
//            }
//            parts_ = best;
//            //parts_.swap(best);

//            // Initialize the deformations
//            for (int i = 0; i < nbParts; ++i)
//                parts_[i + 1].deformation << -0.01, 0.0, -0.01, 0.0, -0.01, 0.0, -0.01, 0.0;
            }
        }

    private:
        std::vector<Model::Part> parts_;
        double bias_;
    };

int main(){

    
    Test t;
//    t.initializeParts();

   

    
    return 0;
}

