//--------------------------------------------------------------------------------------------------
// Written by Fisichella Thomas
// Date 25/05/2018
//
// This file is part of FFLDv2 (the Fast Fourier Linear Detector version 2)
//
// FFLDv2 is free software: you can redistribute it and/or modify it under the terms of the GNU
// Affero General Public License version 3 as published by the Free Software Foundation.
//
// FFLDv2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero
// General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License along with FFLDv2. If
// not, see <http://www.gnu.org/licenses/>.
//--------------------------------------------------------------------------------------------------

#include "Model.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>

using namespace Eigen;
using namespace FFLD;
using namespace std;

Model::Model() : parts_(1), bias_(0.0)
{
	parts_[0].offset.setZero();
	parts_[0].deformation.setZero();
}

Model::Model(Vector3i rootSize, int nbParts, Vector3i partSize) : parts_(1), bias_(0.0)
{
    GSHOTPyramid::Cell nullCell;
    nullCell.setZero( GSHOTPyramid::DescriptorSize);
	parts_[0].offset.setZero();
	parts_[0].deformation.setZero();
	
	// Create an empty model if any of the given parameters is invalid
    if ((rootSize(0) <= 0) || (rootSize(1) <= 0) || (rootSize(2) <= 0) || (nbParts < 0) ||
        ( nbParts && ((partSize(0) <= 0) || (partSize(1) <= 0) || (partSize(2) <= 0)))) {
        cerr << "Attempting to create an empty model : rootSize = "
             << rootSize << " / partSize = " << partSize << " / nbParts = " << nbParts<< endl;
		return;
	}
	
	parts_.resize(nbParts + 1);
	
    parts_[0].filter = GSHOTPyramid::Level(rootSize(0), rootSize(1), rootSize(2));
    parts_[0].filter().setConstant( nullCell);
	
	for (int i = 0; i < nbParts; ++i) {
        parts_[i + 1].filter = GSHOTPyramid::Level(partSize(0), partSize(1), partSize(2));
        parts_[i + 1].filter().setConstant( nullCell);
		parts_[i + 1].offset.setZero();
		parts_[i + 1].deformation.setZero();
	}
}

Model::Model(const vector<Part> & parts, double bias) : parts_(parts), bias_(bias)
{
	// Create an empty model if the parts are empty
	if (parts.empty()) {
		parts_.resize(1);
		parts_[0].offset.setZero();
		parts_[0].deformation.setZero();
	}
}

bool Model::empty() const
{
    //TODO
	return !parts_[0].filter.size() && (parts_.size() == 1);
}

const vector<Model::Part> & Model::parts() const
{
	return parts_;
}

vector<Model::Part> & Model::parts()
{
	return parts_;
}

double Model::bias() const
{
	return bias_;
}

double & Model::bias()
{
	return bias_;
}

Vector3i Model::rootSize() const
{
    return Eigen::Vector3i(parts_[0].filter.depths(),
            parts_[0].filter.rows(),
            parts_[0].filter.cols());
}

Vector3i Model::partSize() const
{
	if (parts_.size() > 1)
        return Eigen::Vector3i(parts_[1].filter.depths(),
                parts_[1].filter.rows(),
                parts_[1].filter.cols());
	else
        return Eigen::Vector3i(0, 0, 0);
}

void Model::initializeParts(int nbParts, Eigen::Vector3i partSize/*, GSHOTPyramid::Level root2x*/)
{
    cout<<"Model::initializeParts ..." <<endl;

    // The model stay unmodified if any of the parameter is invalid
    if (empty() || (nbParts <= 0) || (partSize(0) <= 0) || (partSize(1) <= 0) || (partSize(2) <= 0)) {
        cerr << "Attempting to initialize parts in an empty model" << endl;
        return;
    }


    GSHOTPyramid::Level root = parts_[0].filter;

    GSHOTPyramid::Level root2x( 2*root.depths()-1, 2*root.rows()-1, 2*root.cols()-1);
    GSHOTPyramid::Cell nullCell;
    nullCell.setZero( GSHOTPyramid::DescriptorSize);
    root2x().setConstant(nullCell);

    //Trilinear interpolation
    float xd = 0.5;
    float yd = 0.5;
    float zd = 0.5;
    for (int z = 0; z < root.depths()-1; ++z) {//TODO remove -1 ?
        int zf = z+1;
        for (int y = 0; y < root.rows()-1; ++y) {
            int yf = y+1;
            for (int x = 0; x < root.cols()-1; ++x) {

                int xf = x+1;

                GSHOTPyramid::Cell c00 = root()(z, y, x) * (1-xd) + root()(z, y, xf) * xd;
                GSHOTPyramid::Cell c01 = root()(z, yf, x) * (1-xd) + root()(z, yf, xf) * xd;
                GSHOTPyramid::Cell c10 = root()(zf, y, x) * (1-xd) + root()(zf, y, xf) * xd;
                GSHOTPyramid::Cell c11 = root()(zf, yf, x) * (1-xd) + root()(zf, yf, xf) * xd;

                GSHOTPyramid::Cell c0 = c00 * (1-yd) + c01 * yd;
                GSHOTPyramid::Cell c1 = c10 * (1-yd) + c11 * yd;

                GSHOTPyramid::Cell c = c0 * (1-zd) + c1 * zd;

                root2x()(2*z+1, 2*y+1, 2*x+1) = c;
            }
        }
    }

    for (int z = 0; z < root.depths()/*-1*/; ++z) {
        for (int y = 0; y < root.rows()/*-1*/; ++y) {
            for (int x = 0; x < root.cols()/*-1*/; ++x) {
                root2x()(2*z, 2*y, 2*x) = root()(z, y, x);
            }
        }
    }



    // Compute the energy of each cell
    Tensor3DF energy(root2x.depths(), root2x.rows(), root2x.cols());

    for (int z = 0; z < root2x.depths(); ++z) {
        for (int y = 0; y < root2x.rows(); ++y) {
            for (int x = 0; x < root2x.cols(); ++x) {
                energy()(z, y, x) = 0;

                for (int i = 0; i < GSHOTPyramid::DescriptorSize; ++i)
                    energy()(z, y, x) += root2x()(z, y, x).coeff(i) * root2x()(z, y, x).coeff(i);
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

        cout<<"Model::initializeParts energy.depths() "<<energy.depths() <<endl;
//        cout<<"Model::initializeParts partSize.first "<<partSize.first <<endl;
//        cout<<"Model::initializeParts partSize.second "<<partSize.second <<endl;
//        cout<<"Model::initializeParts partSize.third "<<partSize.third <<endl;

        for (int z = 0; z <= energy.depths() - partSize(0); ++z) {
            for (int y = 0; y <= energy.rows() - partSize(1); ++y) {
                for (int x = 0; x <= energy.cols() - partSize(2); ++x) {
                    const double e = energy.block(z, y, x, partSize(0), partSize(1), partSize(2)).sum();

                    if (e > maxEnergy) {
                        maxEnergy = e;
                        argX = x;
                        argY = y;
                        argZ = z;
                    }
                }
            }
        }

        // Initialize the part
        parts_[i + 1].filter = root2x.block(argZ, argY, argX, partSize(0), partSize(1), partSize(2));
        parts_[i + 1].offset(0) = argZ;
        parts_[i + 1].offset(1) = argY;
        parts_[i + 1].offset(2) = argX;

        cout<<"Model::initializeParts init part 1 : offset["<<i+1<<"] = "<< parts_[i+1].offset <<endl;

        // Set the energy of the part to zero
        energy.blockLink(argZ, argY, argX, partSize(0), partSize(1), partSize(2)).setZero();
    }
////////////
//    // Retry 10 times from randomized starting points
//    double bestCover = 0.0;
//    vector<Part> best(parts_); // The best so far is the current one

//    for (int i = 0; i < 10; ++i) {
//        vector<Part> tmp(parts_); // Try from the current one
////        cout << "size tmp "<<tmp.size()<<endl;
//        // Remove a part at random and look for the best place to put it
//        for (int j = 0; j < 100 * nbParts; ++j) {
//            // Recompute the energy
//            for (int z = 0; z < root2x.depths(); ++z) {
//                for (int y = 0; y < root2x.rows(); ++y) {
//                    for (int x = 0; x < root2x.cols(); ++x) {
//                        energy()(z, y, x) = 0;

//                        for (int i = 0; i < GSHOTPyramid::DescriptorSize; ++i)
//                            energy()(z, y, x) += root2x()(z, y, x).coeff(i) * root2x()(z, y, x).coeff(i);
//                    }
//                }
//            }

//            // Select a part at random
//            const int part = rand() % nbParts;

//            // Zero out the energy covered by the other parts
//            for (int k = 0; k < nbParts; ++k){
//                if (k != part){
////                    cout<<"Model::initializeParts set energyy Zero at = "<< tmp[k+1].offset <<endl;
//                    energy.blockLink(tmp[k + 1].offset(0), tmp[k + 1].offset(1), tmp[k + 1].offset(2), partSize(0),
//                                 partSize(1), partSize(2)).setZero();

////                    const double e = energy.block(tmp[k + 1].offset(0), tmp[k + 1].offset(1), tmp[k + 1].offset(2),
////                            partSize.first,partSize.second,partSize.third).sum();
////                    cout<<"Model::initializeParts energy at "<<tmp[k + 1].offset(0)<<" / "<<
////                       tmp[k + 1].offset(1)<<" / "<<tmp[k + 1].offset(2)<<" = "<< e <<endl;

//                }
//            }

//            // Find the region of maximum energy
//            double maxEnergy = 0.0;
//            int argX = 0;
//            int argY = 0;
//            int argZ = 0;

//            for (int z = 0; z <= energy.depths() - partSize(0); ++z) {
//                for (int y = 0; y <= energy.rows() - partSize(1); ++y) {
//                    for (int x = 0; x <= energy.cols() - partSize(2); ++x) {
//                        const double e = energy.block(z, y, x,partSize(0), partSize(1), partSize(2)).sum();
////                        cout<<"Model::initializeParts energy at "<<z<<" / "<<y<<" / "<<x<<" = "<< e <<endl;

//                        if (e > maxEnergy) {
//                            maxEnergy = e;
//                            argX = x;
//                            argY = y;
//                            argZ = z;
//                        }
////                        cout<<"Model::initializeParts energy at "<<z<<" / "<<y<<" / "<<x<<" = "<< e <<" / "<<maxEnergy<<endl;
//                    }
//                }
//            }

//            // Initialize the part
//            tmp[part + 1].filter = root2x.block(argZ, argY, argX, partSize(0), partSize(1), partSize(2));
//            tmp[part + 1].offset(0) = argZ;
//            tmp[part + 1].offset(1) = argY;
//            tmp[part + 1].offset(2) = argX;

//            cout<<"Model::initializeParts init part 2 : offset["<<part+1<<"] = "<< tmp[part+1].offset <<endl;

//        }

//        // Compute the energy covered by this part arrangement
//        double cover = 0.0;

//        // Recompute the energy
//        for (int z = 0; z < root2x.depths(); ++z) {
//            for (int y = 0; y < root2x.rows(); ++y) {
//                for (int x = 0; x < root2x.cols(); ++x) {
//                    energy()(z, y, x) = 0;

//                    for (int i = 0; i < GSHOTPyramid::DescriptorSize; ++i)
//                        energy()(z, y, x) += root2x()(z, y, x).coeff(i) * root2x()(z, y, x).coeff(i);
//                }
//            }
//        }

//        for (int j = 0; j < nbParts; ++j) {
////            cout << energy.size() <<endl;
//            // Add the energy of the part
//            cover += energy.block(tmp[j + 1].offset(0), tmp[j + 1].offset(1), tmp[j + 1].offset(2),
//                                  partSize(0), partSize(1), partSize(2)).sum();

//            // Set the energy of the part to zero
//            energy.blockLink(tmp[j + 1].offset(0), tmp[j + 1].offset(1), tmp[j + 1].offset(2),
//                    partSize(0), partSize(1), partSize(2)).setZero();
//        }

//        if (cover > bestCover) {
//            bestCover = cover;
//            best.swap(tmp);
//        }
//    }

//    //error seg
//    parts_.swap(best);
//////////////

    // Initialize the deformations
    //TODO replace values see paper
    for (int i = 0; i < nbParts; ++i)
        parts_[i + 1].deformation << -0.01, 0.0, -0.01, 0.0, -0.01, 0.0, -0.01, 0.0;

    cout<<"Model::initializeParts done" <<endl;

}


void Model::initializeSample(const GSHOTPyramid & pyramid, int box, int z, int y, int x, int lvl, Model & sample,
                             const vector<vector<vector<Positions> > > *positions) const
{
//    cout << "initializeSample ..." << endl;
    // All the constants relative to the model and the pyramid
    const int nbFilters = static_cast<int>(parts_.size());
    const int nbParts = nbFilters - 1;
    const int interval = pyramid.interval();
    const int nbLevels = static_cast<int>(pyramid.levels().size());
	
//    cout << "initializeSample pyramid.levels()["<<lvl<<"].cols() = " << pyramid.levels()[lvl].cols() << endl;

    // Invalid parameters
    if (empty() || (x < 0) || (y < 0) || (z < 0) || (lvl >= nbLevels) ||
        (x + rootSize()(2) > pyramid.levels()[lvl][box].cols()) ||//////TODO (rootSize-1)*....
        (y + rootSize()(1) > pyramid.levels()[lvl][box].rows()) ||
        (z + rootSize()(0) > pyramid.levels()[lvl][box].depths()) ||
        (nbParts && (!positions || (positions->size() != nbParts)))) {
        sample = Model();
        cerr << "Attempting to initialize an empty sample 1" << endl;
        cerr << "lvl : "<<lvl<<" >= "<<nbLevels<< endl;
        cerr << "x : "<<x + rootSize()(2)<<" > "<<pyramid.levels()[lvl][box].cols()<< endl;
        cerr << "y : "<<y + rootSize()(1)<<" > "<<pyramid.levels()[lvl][box].rows()<< endl;
        cerr << "z : "<<z + rootSize()(0)<<" > "<<pyramid.levels()[lvl][box].depths()<< endl;
        cerr << "rootSize : "<<rootSize()(0)<<" / "<<rootSize()(1)<<" / "<<rootSize()(2)<< endl;
        cerr << "empty() : "<<empty()<< endl;
        if(nbParts && (!positions || (positions->size() != nbParts))) {
            cerr << "positions : "<<positions<< endl;
//            cerr << "nbParts : "<<nbParts<<" =  positions->size() :"<<positions->size()<< endl;
        }
        return;
    }
	
    // Resize the sample to have the same number of filters as the model
    sample.parts_.resize(nbFilters);
	
//    cout << "initializeSample coord = " << z<<" / "<<y<<" / "<<x <<" / "<< lvl<< endl;

    // Extract the root filter

//    Eigen::array<int, 3> offsets = {z, y, x};
//    Eigen::array<int, 3> extents = {1,1,1};
//    Eigen::array<int, 3> three_dims{{rootSize().first, rootSize().second, rootSize().third}};

//    sample.parts_[0].filter() = pyramid.levels()[lvl]().slice(offsets, extents).reshape(three_dims);


    sample.parts_[0].filter = pyramid.levels()[lvl][box].block(z, y, x, rootSize()(0),
                                                          rootSize()(1), rootSize()(2));
    sample.parts_[0].offset.setZero();
    sample.parts_[0].deformation.setZero();

//TODO parallelize
    for (int i = 0; i < nbParts; ++i) {
        // Position of the part
        if ((lvl >= (*positions)[i].size()) || (x >= (*positions)[i][lvl][box].cols()) ||
            (y >= (*positions)[i][lvl][box].rows()) || (z >= (*positions)[i][lvl][box].depths())) {
            sample = Model();
            cerr << "Attempting to initialize an empty sample 2" << endl;
            cerr << "lvl : "<<lvl<<" >= "<<(*positions)[i].size()<< endl;
            cerr << "x : "<<x<<" >= "<<(*positions)[i][lvl][box].cols()<< endl;
            cerr << "y : "<<y<<" >= "<<(*positions)[i][lvl][box].rows()<< endl;
            cerr << "z : "<<z<<" >= "<<(*positions)[i][lvl][box].depths()<< endl;

            return;
        }
		
        const Position position = (*positions)[i][lvl][box]()(z, y, x);
		
        // Level of the part
        if ((position(3) < 0) || (position(3) >= nbLevels)) {
            sample = Model();
            cerr << "Attempting to initialize an empty sample 3" << endl;
            return;
        }
		
        const GSHOTPyramid::Level & level = pyramid.levels()[position(3)][box];
		
        if ((position(0) < 0) || (position(1) < 0 || (position(2) < 0))) {
            sample = Model();
            cerr << "Attempting to initialize an empty sample 4 : " << position << endl;
            return;
        }
        if ((position(2) + partSize()(2) > level.cols()) ||
            (position(1) + partSize()(1) > level.rows()) ||
            (position(0) + partSize()(0) > level.depths())) {
            sample = Model();
            cerr << "Attempting to initialize an empty sample 5 : " << level.depths()
                 << " " << level.rows() << " " << level.cols()<< endl;
            return;
        }
		
//        cout<<"Model::InitiSample filter part isZero = "<<GSHOTPyramid::TensorMap(level).isZero()<<endl;
//        cout<<"Model::InitiSample filter part block isZero = "<<GSHOTPyramid::TensorMap(
//                  level.block(position(0),
//                                position(1),
//                                position(2),
//                                partSize().first,
//                                partSize().second,
//                                partSize().third)).isZero()<<endl;

        // Extract the part filter
        sample.parts_[i + 1].filter = level.block(position(0), position(1), position(2), partSize()(0),
                                                  partSize()(1), partSize()(2));
		
        // Set the part offset to the position
        sample.parts_[i + 1].offset = position;

//        cout<<"Model::InitiSample filter part position = "<<position<<endl;

		
        //TODO!!!!
        // Compute the deformation gradient at the level of the part
//        const double scale = 1;//pow(2.0, static_cast<double>(lvl - position(3)) / interval);
//        const double xr = (x + (parts_[i + 1].offset(2) + partSize().third * 0.5) * 0.5 - pad.x()) *
//                          scale + pad.x() - partSize().third * 0.5;
//        const double yr = (y + (parts_[i + 1].offset(1) + partSize().second * 0.5) * 0.5 - pad.y()) *
//                          scale + pad.y() - partSize().second * 0.5;
//        const double zr = (z + (parts_[i + 1].offset(0) + partSize().first * 0.5) * 0.5 - pad.z()) *
//                          scale + pad.z() - partSize().first * 0.5;
        const double scale = pow(2.0, static_cast<double>(lvl - position(3)) / interval);
        const double xr = (x + (parts_[i + 1].offset(2) + partSize()(2) * 0.5) * 0.5 /*- pad.x()*/) *
                          scale + /*pad.x()*/ - partSize()(2) * 0.5;
        const double yr = (y + (parts_[i + 1].offset(1) + partSize()(1) * 0.5) * 0.5 /*- pad.y()*/) *
                          scale + /*pad.y()*/ - partSize()(1) * 0.5;
        const double zr = (z + (parts_[i + 1].offset(0) + partSize()(0) * 0.5) * 0.5 /*- pad.z()*/) *
                          scale + /*pad.z()*/ - partSize()(0) * 0.5;
        const double dx = xr - position(2)/**pyramid.resolutions()[lvl]*/;
        const double dy = yr - position(1)/**pyramid.resolutions()[lvl]*/;
        const double dz = zr - position(0)/**pyramid.resolutions()[lvl]*/;
        const int dlvl = lvl - interval - position(3);
		
        sample.parts_[i + 1].deformation(0) = dz * dz;
        sample.parts_[i + 1].deformation(1) = dz;
        sample.parts_[i + 1].deformation(2) = dy * dy;
        sample.parts_[i + 1].deformation(3) = dy;
        sample.parts_[i + 1].deformation(4) = dx * dx;
        sample.parts_[i + 1].deformation(5) = dx;
        sample.parts_[i + 1].deformation(6) = dlvl * dlvl;
        sample.parts_[i + 1].deformation(7) = dlvl;
    }
	
    sample.bias_ = 1.0;
}

void Model::convolve(const GSHOTPyramid & pyramid, vector<vector<Tensor3DF> > &scores,//[lvl][box]
                     vector<vector<vector<Positions> > >* positions,//[part][lvl][box]
                     vector<vector<vector<Tensor3DF> > > * convolutions/*useless*/) const
{

	// Invalid parameters
    if (empty() || pyramid.empty() /*||*/
#ifdef FFLD_MODEL_3D
//		!positions ||
#endif
        /*(convolutions && (convolutions->size() != parts_.size()))*/) {
        if(empty()) cout<<"Model::convolve model is empty "<<endl;
        else cout<<"Model::convolve pyramid is empty "<<endl;
        scores.clear();
        cout<<"Model::scores cleared "<<endl;
		
		if (positions)
			positions->clear();
		
		return;
	}
	
	// All the constants relative to the model and the pyramid
	const int nbFilters = static_cast<int>(parts_.size());
	const int nbParts = nbFilters - 1;
	const int interval = pyramid.interval();
	const int nbLevels = static_cast<int>(pyramid.levels().size());
	
	// Convolve the pyramid with all the filters
    vector<vector<vector<Tensor3DF> > >tmpConvolutions;
	
	if (convolutions) {
		for (int i = 0; i < nbFilters; ++i) {
			if ((*convolutions)[i].size() != nbLevels) {
				scores.clear();
                cout<<"Model::convolve score clear"<<endl;
				
				if (positions)
					positions->clear();
				
				return;
			}
		}
	}
	else {
		tmpConvolutions.resize(nbFilters);
		
    cout<<"Model::convolve ..."<<endl;

#pragma omp parallel for
        for (int i = 0; i < nbFilters; ++i){

            pyramid.convolve(parts_[i].filter, tmpConvolutions[i]);
//            cout<<"Model::convolve pyramid convolution results of size : "<< tmpConvolutions.size()
//               << " / " << tmpConvolutions[i].size() << " / " << tmpConvolutions[i][0].size()<<endl;
//            cout<<"Model::convolve tmpConvolutions["<<i<<"][0].isZero() : "<< tmpConvolutions[i][0].isZero()<<endl;
//            cout<<"Model::convolve tmpConvolutions["<<i<<"][1].isZero() : "<< tmpConvolutions[i][1].isZero()<<endl;

        }

        convolutions = &tmpConvolutions;//[part][lvl][box]
	}
	
    // Resize the positions
    if (positions) {
        cout<<"Model::convolve resize positions"<<endl;

        positions->resize(nbParts);
		
        for (int i = 0; i < nbParts; ++i){
            (*positions)[i].resize(nbLevels);
            for (int j = 0; j < pyramid.levels().size(); ++j){
                (*positions)[i][j].resize(pyramid.levels()[j].size());
            }
        }
    }
	
    // Temporary data needed by the distance transforms
    Tensor3DF tmp1;
    Tensor3DF tmp2;


    float sum = 0.0;
	
    // For each root level in reverse order
    for (int lvl = nbLevels - 1; lvl >= interval; --lvl) {

//        if((*convolutions)[0][lvl - interval].size() > 0){
//            stringstream name;
//            name << "conv" << 0 << ".txt";
//            ofstream out(name.str().c_str());
//            out << (*convolutions)[0][lvl]();
//        }
        for (int box = 0; box < pyramid.levels()[lvl].size(); ++box){
//            cout<<"Model::conv score start : "<<(*convolutions)[0][lvl][box]()(0,0,0)<<endl;

            // For each part
            for (int i = 0; i < nbParts; ++i) {


                // Transform the part one octave below

    //            sum = (*convolutions)[i+1][lvl - interval].sum();
    //            cout<<"Model::conv sum for part "<<i+1<<" before DT3D : "<<sum<<endl;

                DT3D((*convolutions)[i + 1][lvl - interval][box], parts_[i + 1], tmp1, tmp2,
                     positions ? &(*positions)[i][lvl - interval][box] : 0);

                if (positions){
                    (*positions)[i][lvl][box] = Positions((*convolutions)[0][lvl][box].depths(),
                                                     (*convolutions)[0][lvl][box].rows(),
                                                     (*convolutions)[0][lvl][box].cols());
                    (*positions)[i][lvl][box]().setConstant( Position::Zero());
                }

    //            sum = (*convolutions)[i+1][lvl - interval].sum();
    //            cout<<"Model::conv sum for part "<<i+1<<" after DT3D : "<<sum<<endl;

                if((*convolutions)[i+1][lvl - interval][box].size() > 0 && box == 403){
                    stringstream name;
                    name << "conv" << i+1 << ".txt";
                    ofstream out(name.str().c_str());
                    out << (*convolutions)[i+1][lvl - interval][box]();
                }

                // Add the distance transforms of the part one octave below
                for (int z = 0; z < (*convolutions)[0][lvl][box].depths(); ++z) {//lvl=1
                    for (int y = 0; y < (*convolutions)[0][lvl][box].rows(); ++y) {
                        for (int x = 0; x < (*convolutions)[0][lvl][box].cols(); ++x) {

                            const int zr = 2 * z/*- pad.z()*/ + parts_[i + 1].offset(0);//coord lvl - interval 0
                            const int yr = 2 * y /*- pad.y()*/ + parts_[i + 1].offset(1);
                            const int xr = 2 * x /*- pad.x()*/ + parts_[i + 1].offset(2);

                            if ((xr >= 0) && (yr >= 0) && (zr >= 0) &&
                                (xr < (*convolutions)[i + 1][lvl - interval][box].cols()) &&//lvl - interval 0
                                (yr < (*convolutions)[i + 1][lvl - interval][box].rows()) &&
                                (zr < (*convolutions)[i + 1][lvl - interval][box].depths())) {

                                (*convolutions)[0][lvl][box]()(z, y, x) += (*convolutions)[i + 1][lvl - interval][box]()(zr, yr, xr);
//                                cout<<"Model::conv score : "<<(*convolutions)[0][lvl][box]()(0,0,0)<<endl;

                                if (positions){
                                    (*positions)[i][lvl][box]()(z, y, x) <<
                                        (*positions)[i][lvl - interval][box]()(zr, yr, xr)(0),
                                        (*positions)[i][lvl - interval][box]()(zr, yr, xr)(1),
                                        (*positions)[i][lvl - interval][box]()(zr, yr, xr)(2),
                                        lvl - interval;
//                                    cout<<"Model::(*positions)[i][lvl - interval][box]()(zr, yr, xr) "<<(*positions)[i][lvl - interval][box]()(zr, yr, xr)<<endl;
                                }
                            }
                            else {
    //                            cout<<"Model::conv set score to -inf ..."<<endl;
    //                            cout<<"Model::conv "<<zr<<" < "<<(*convolutions)[i + 1][lvl - interval].depths()<<endl;
    //                            cout<<"Model::conv "<<yr<<" < "<<(*convolutions)[i + 1][lvl - interval].rows()<<endl;
    //                            cout<<"Model::conv "<<xr<<" < "<<(*convolutions)[i + 1][lvl - interval].cols()<<endl;

                                (*convolutions)[0][lvl][box]()(z, y, x) =
                                    -numeric_limits<GSHOTPyramid::Scalar>::infinity();
                            }
                        }
                    }
                }
                if (positions)
                    (*positions)[i][lvl - interval][box] = Positions();


            }
//            cout<<"Model::conv score end : "<<(*convolutions)[0][lvl][box]()(0,0,0)<<endl;
//            (*convolutions)[0][lvl][box]()(0,0,0) /=  nbFilters;
//            cout<<"Model::conv score final : "<<(*convolutions)[0][lvl][box]()(0,0,0)<<endl;

        }

    }

    scores.swap((*convolutions)[0]);

    for (int i = 0; i < interval; ++i) {
        for (int box = 0; box < pyramid.levels()[0].size(); ++box){

            scores[i][box] = Tensor3DF();

            for (int j = 0; j < nbParts; ++j){
                (*positions)[j][i][box] = Positions();
            }
        }
    }

//    float den = 1.0/nbFilters;
//    cout<<"Model::conv den : "<<den<<" / nbFilters : "<<nbFilters<<endl;
//    scores[1] *= den;

//     Add the bias if necessary
    if (bias_) {
//#pragma omp parallel for
        for (int i = interval; i < nbLevels; ++i){
            for (int box = 0; box < pyramid.levels()[0].size(); ++box){

    //			scores[i].array() += bias_;
                Tensor3DF biasTensor( scores[i][box].depths(), scores[i][box].rows(), scores[i][box].cols());
                biasTensor().setConstant( bias_);
                scores[i][box]() += biasTensor();
//                scores[i][box]()(0,0,0) /=  nbFilters;
            }
        }
    }

}

double Model::dot(const Model & sample) const
{
    double d = bias_ * sample.bias_;

    if (parts_.size() != sample.parts_.size()){
        cout<<"Model::dot bug1"<< endl;
        return numeric_limits<double>::quiet_NaN();
    }

    for (int i = 0; i < parts_.size(); ++i) {
        if ((parts_[i].filter.depths() != sample.parts_[i].filter.depths()) ||
            (parts_[i].filter.rows() != sample.parts_[i].filter.rows()) ||
            (parts_[i].filter.cols() != sample.parts_[i].filter.cols())){
            cout<<"Model::dot bug2"<< endl;
            return numeric_limits<double>::quiet_NaN();
        }

//        GSHOTPyramid::Level f1 = parts_[i].filter.agglomerate();
//        GSHOTPyramid::Level f2 = sample.parts_[i].filter.agglomerate();

//        for (int j = 0; j < GSHOTPyramid::DescriptorSize; ++j){;
//            d += f1()(0,0,0)(j) * f2()(0,0,0)(j);
//        }

        for (int z = 0; z < parts_[i].filter.depths(); ++z){
            for (int y = 0; y < parts_[i].filter.rows(); ++y){
                for (int x = 0; x < parts_[i].filter.cols(); ++x){
                    for (int j = 0; j < GSHOTPyramid::DescriptorSize; ++j){
                        d += parts_[i].filter()(z, y, x)(j) * sample.parts_[i].filter()(z, y, x)(j);
                    }
                }
            }
        }

        if (i){
            for (int j = 0; j < parts_[i].deformation.size(); ++j){
                d += parts_[i].deformation(j) * sample.parts_[i].deformation(j);
            }
        }
    }
	
	return d;
}

double Model::norm() const{

    if( parts_.size() < 1){
        return GSHOTPyramid::TensorMap(parts_[0].filter).squaredNorm();
    } else{
        double n = GSHOTPyramid::TensorMap(parts_[0].filter).squaredNorm();

        for (int i = 1; i < parts_.size(); ++i) {
            n += GSHOTPyramid::TensorMap(parts_[i].filter).squaredNorm();


            for(int j = 0; j < parts_[i].deformation.size(); ++j){
                n += 10 * parts_[i].deformation(j) * parts_[i].deformation(j);
            }
        }
        return sqrt(n);
    }
}

Model & Model::operator+=(const Model & sample)
{
	if (parts_.size() != sample.parts_.size())
		return *this;
	
	Model copy(*this);
	
	for (int i = 0; i < parts_.size(); ++i) {
        if ((parts_[i].filter.depths() != sample.parts_[i].filter.depths()) ||
            (parts_[i].filter.rows() != sample.parts_[i].filter.rows()) ||
			(parts_[i].filter.cols() != sample.parts_[i].filter.cols())) {
			*this = copy; // Restore the copy
			return *this;
		}
		
        parts_[i].filter() += sample.parts_[i].filter();
		parts_[i].deformation += sample.parts_[i].deformation;
	}
	
	bias_ += sample.bias_;
	
	return *this;
}

Model & Model::operator-=(const Model & sample)
{
	if (parts_.size() != sample.parts_.size())
		return *this;
	
	Model copy(*this);
	
	for (int i = 0; i < parts_.size(); ++i) {
        if ((parts_[i].filter.depths() != sample.parts_[i].filter.depths()) ||
            (parts_[i].filter.rows() != sample.parts_[i].filter.rows()) ||
            (parts_[i].filter.cols() != sample.parts_[i].filter.cols())) {
			*this = copy; // Restore the copy
			return *this;
		}
		
        parts_[i].filter() -= sample.parts_[i].filter();
		parts_[i].deformation -= sample.parts_[i].deformation;
	}
	
	bias_ -= sample.bias_;
	
	return *this;
}

Model & Model::operator*=(double a)
{
    for (int i = 0; i < parts_.size(); ++i) {
        for (int z = 0; z < parts_[i].filter.depths(); ++z){
            for (int y = 0; y < parts_[i].filter.rows(); ++y){
                for (int x = 0; x < parts_[i].filter.cols(); ++x){
                    for (int j = 0; j < GSHOTPyramid::DescriptorSize; ++j){
                        parts_[i].filter()(z, y, x)(j) *= a;
                    }
                }
            }
        }
        parts_[i].deformation *= a;
    }

    bias_ *= a;
	
    return *this;
}

// y = EDT result
// m = positions
// z = distance
// off = offset used to shift the weight grid to the right location of the part disregard to the root location p
template <typename Scalar>
static void dt1d(const Scalar * f, int n, Scalar a, Scalar b, Scalar * z, int * v, Scalar * y,
                 int * m, const Scalar * t, int incf, int incy, int incm, int off = 0, int weightSize = 10)
{
    assert(f && (y || m));
    assert(n > 0);
    assert(a < 0);
    assert(z && v);
    assert(t);
    assert(incf && incy && (m ? incm : true));


    int start = std::max(0, off - weightSize);
    int end = std::min(n, off + weightSize);

    z[0] =-numeric_limits<Scalar>::infinity();  //Locations of boundaries between parabolas
    z[1] = numeric_limits<Scalar>::infinity();  //Locations of boundaries between parabolas
    v[0] = 0;                                   //Locations of parabolas in lower envelope

    // Use a lookup table to replace the division by (a * (q - v[k]))
    int k = 0;                                  //Index of rightmost parabola in lower envelope
    Scalar fvk = f[0];



    for (int q = 1; q < n;) {
        const Scalar s = (f[q * incf] - fvk) * t[q - v[k]] + (q + v[k]) - b / a;
        // result good only if b = 0
//        const Scalar s = (f[q * incf] - fvk) * t[q - v[k]] / 2.0 + (q*q - v[k]*v[k]) / (2*(q - v[k])) - off
//                - b * (q - v[k]) / a;
//        const Scalar s = (x[q * incx] - xvk) * t[q - v[k]] + (q + v[k]) - off;

        if (s <= z[k]) {
            --k;
            fvk = f[v[k] * incf];
        }
        else {
            ++k;
            v[k] = q;
            z[k] = s;
            z[k + 1] = numeric_limits<Scalar>::infinity();
            fvk = f[q * incf];
            ++q;
        }
    }

    z[k + 1] = numeric_limits<Scalar>::infinity();

    if (y && m) {
        for (int q = 0, k = 0; q < n; ++q) {
            while (z[k + 1] < 2 * q)
                ++k;

            y[q * incy] = f[v[k] * incf] + (a * (q - v[k]) + b) * (q - v[k]);
            m[q * incm] = v[k];
//            if(q >= start && q <= end)
//                y[q * incy] = f[v[k] * incf] + (a * (q - v[k] + off) + b) * (q - v[k] + off);
//            else
//                y[q * incy] = 0;//f[v[k] * incf];
//            m[q * incm] = v[k] ;
        }
    }
    else if (y) {
        for (int q = 0, k = 0; q < n; ++q) {
            while (z[k + 1] < 2 * q)
                ++k;

            y[q * incy] = f[v[k] * incf] + (a * (q - v[k]) + b) * (q - v[k]);
//            if(q >= start && q <= end)
//                y[q * incy] = f[v[k] * incf] + (a * (q - v[k] + off) + b) * (q - v[k] + off);
//            else
//                y[q * incy] = 0;//f[v[k] * incf];
        }
    }
    else {
        for (int q = 0, k = 0; q < n; ++q) {
            while (z[k + 1] < 2 * q)
                ++k;

//            m[q * incm] = v[k];
            m[q * incm] = v[k] ;
        }
    }
}

//TODO init tmp1 and 2 in the function and allow b != 0
// Tensor = convolution score of the parts in the scene
void Model::DT3D(Tensor3DF & tensor, const Part & part, Tensor3DF & tmp1, Tensor3DF & tmp2,
                 Positions * positions)
{
    // Nothing to do if the matrix is empty
    if (!tensor.size())
        return;

//    cout<<"Model::DT3D offsets : "<<part.offset<<endl;

    const int depths = static_cast<int>(tensor.depths());
    const int rows = static_cast<int>(tensor.rows());
    const int cols = static_cast<int>(tensor.cols());

    Tensor3DF copy(depths, rows, cols);
    for (int z = 0; z < depths; ++z){
        for (int y = 0; y < rows; ++y){
            for (int x = 0; x < cols; ++x){
                copy()(z, y, x) = tensor()(z, y, x);
            }
        }
    }

//    cout<<"Model::DT3D begin max : "<<copy.max()<<endl;
//    cout<<"Model::DT3D begin min : "<<copy.min()<<endl;


    ///TODO : doesnt matter ???
    const int weightDepths = 5;
    const int weightRows   = 5;
    const int weightCols   = 5;
//    const int weightDepths = depths;
//    const int weightRows   = rows;
//    const int weightCols   = cols;

    if (positions){
        (*positions)().resize(depths, rows, cols);
//        (*positions)().setZero();
    }

    tmp1().resize(depths, rows, cols);
    tmp2().resize(depths, rows, cols);

    // Temporary vectors
    int maxi = max(depths,max(rows, cols));
    vector<GSHOTPyramid::Scalar> distance(maxi + 1);
    vector<int> index(maxi + 1);
    vector<GSHOTPyramid::Scalar> t(maxi);

    t[0] = numeric_limits<GSHOTPyramid::Scalar>::infinity();

    for (int x = 1; x < cols; ++x){
        t[x] = 1 / (part.deformation(0) * x);
    }
    //dt1d(const Scalar * x, int n, Scalar a, Scalar b, Scalar * z, int * v, Scalar * y,
    //int * m, const Scalar * t, int incx, int incy, int incm)

    // Filter the rows in tmp
    for (int z = 0; z < depths; ++z){
        for (int y = 0; y < rows; ++y){
            dt1d<GSHOTPyramid::Scalar>(tensor().data() + y*cols + z*cols*rows, cols, part.deformation(0),
                                 part.deformation(1), &distance[0], &index[0], tmp1().data() + y*cols + z*cols*rows,
                                 positions ? ((*positions)().data() + y*cols + z*cols*rows)->data() + 2 : 0,
                                 &t[0], 1, 1, 4);
//            dt1d<GSHOTPyramid::Scalar>(tensor().data() + y*cols + z*cols*rows, cols, part.deformation(0),
//                                 0, &distance[0], &index[0], tmp1().data() + y*cols + z*cols*rows,
//                                 positions ? ((*positions)().data() + y*cols + z*cols*rows)->data() + 2 : 0,
//                                 &t[0], 1, 1, 4, part.offset[2], weightCols);
        }
    }


    for (int y = 1; y < rows; ++y){
        t[y] = 1 / (part.deformation(2) * y);
    }

    // Filter the columns back to the original matrix
    for (int z = 0; z < depths; ++z){
        for (int x = 0; x < cols; ++x){
            dt1d<GSHOTPyramid::Scalar>(tmp1().data() + x + z*cols*rows, rows, part.deformation(2), part.deformation(3),
                                 &distance[0], &index[0],
                                 tmp2().data() + x + z*cols*rows,
                                 positions ? ((*positions)().data() + x + z*cols*rows)->data() + 1 : 0, &t[0],
                                 cols, cols, 4 * cols);
//            dt1d<GSHOTPyramid::Scalar>(tmp1().data() + x + z*cols*rows, rows, part.deformation(2), 0,
//                                 &distance[0], &index[0],
//                                 tmp2().data() + x + z*cols*rows,
//                                 positions ? ((*positions)().data() + x + z*cols*rows)->data() + 1 : 0, &t[0],
//                                 cols, cols, 4 * cols, part.offset[1], weightRows);
        }
    }


    for (int z = 1; z < depths; ++z){
        t[z] = 1 / (part.deformation(4) * z);
    }

    // Filter the columns back to the original matrix
    for (int y = 0; y < rows; ++y){
        for (int x = 0; x < cols; ++x){
            dt1d<GSHOTPyramid::Scalar>(tmp2().data() + x + y*cols, depths, part.deformation(4), part.deformation(5),
                                 &distance[0], &index[0],
                                 tensor().data() + x + y*cols,//or 0
                                 positions ? ((*positions)().data() + x + y*cols)->data() : 0, &t[0],
                                 cols * rows, cols * rows, 4 * cols * rows);
//            dt1d<GSHOTPyramid::Scalar>(tmp2().data() + x + y*cols, depths, part.deformation(4), 0,
//                                 &distance[0], &index[0],
//                                 tensor().data() + x + y*cols,//or 0
//                                 positions ? ((*positions)().data() + x + y*cols)->data() : 0, &t[0],
//                                 cols * rows, cols * rows, 4 * cols * rows, part.offset[0], weightDepths);
        }
    }

//    // Re-index the best z positions now that the best x and y changed
    if (positions) {
        for (int z = 0; z < depths; ++z)
            for (int y = 0; y < rows; ++y)
                for (int x = 0; x < cols; ++x){
                    tmp1()(z, y, x) = (*positions)()(z, y, x)(2);
                    tmp2()(z, y, x) = (*positions)()(z, y, x)(1);
                }

        for (int z = 0; z < depths; ++z)
            for (int y = 0; y < rows; ++y)
                for (int x = 0; x < cols; ++x){

                    (*positions)()(z, y, x)(2) = tmp1()( (*positions)()(z, y, x)(0), (*positions)()(z, y, x)(1), x);
                    (*positions)()(z, y, x)(1) = tmp2()((*positions)()(z, y, x)(0), y, (*positions)()(z, y, x)(2));
                }

    }

//    cout<<"Model::DT3D end max : "<<tensor.max()<<endl;
//    cout<<"Model::DT3D end min : "<<tensor.min()<<endl;


}

ostream & FFLD::operator<<(ostream & os, const Model & model)
{
	// Save the number of parts and the bias
	os << model.parts().size() << ' ' << model.bias() << endl;
	
	// Save the parts themselves
	for (int i = 0; i < model.parts().size(); ++i) {
        os << model.parts()[i].filter.depths() << ' ' << model.parts()[i].filter.rows()
           << ' ' << model.parts()[i].filter.cols() << ' ' << GSHOTPyramid::DescriptorSize
           << ' ' << model.parts()[i].offset(0) << ' ' << model.parts()[i].offset(1)
           << ' ' << model.parts()[i].offset(2) << ' ' << model.parts()[i].offset(3) << ' '
		   << model.parts()[i].deformation(0) << ' ' << model.parts()[i].deformation(1) << ' '
		   << model.parts()[i].deformation(2) << ' ' << model.parts()[i].deformation(3) << ' '
           << model.parts()[i].deformation(4) << ' ' << model.parts()[i].deformation(5) << ' '
           << model.parts()[i].deformation(6) << ' ' << model.parts()[i].deformation(7)
		   << endl;

        for (int z = 0; z < model.parts()[i].filter.depths(); ++z) {
            for (int y = 0; y < model.parts()[i].filter.rows(); ++y) {
                os << model.parts()[i].filter()(z, y, 0)(0);
                for (int j = 1; j < GSHOTPyramid::DescriptorSize; ++j)
                    os << ' ' << model.parts()[i].filter()(z, y, 0)(j);

                for (int x = 1; x < model.parts()[i].filter.cols(); ++x)
                    for (int j = 0; j < GSHOTPyramid::DescriptorSize; ++j)
                        os << ' ' << model.parts()[i].filter()(z, y, x)(j);

                os << endl;
            }
        }

	}
	
	return os;
}

istream & FFLD::operator>>(istream & is, Model & model)
{

	int nbParts;
	double bias;
	
	is >> nbParts >> bias;
	
	if (!is) {
        cerr<<"Model::operator>> failed 1"<<endl;
		model = Model();
		return is;
	}
	
	vector<Model::Part> parts(nbParts);

    for (int i = 0; i < nbParts; ++i) {
        int depths, rows, cols, nbFeatures;		
        is >> depths >> rows >> cols >> nbFeatures;
        is >> parts[i].offset(0) >> parts[i].offset(1)
           >> parts[i].offset(2) >> parts[i].offset(3);
        is >> parts[i].deformation(0) >> parts[i].deformation(1)
           >> parts[i].deformation(2) >> parts[i].deformation(3) >> parts[i].deformation(4)
           >> parts[i].deformation(5) >> parts[i].deformation(6) >> parts[i].deformation(7);
		
        if (!is || (nbFeatures > GSHOTPyramid::DescriptorSize)) {
            if(!is)cerr<<"Model::operator>> failed 2: stream is empty"<<endl;
            else cerr<<"Model::operator>> failed 2: nbFeatures: "<<nbFeatures<<" > DescriptorSize:"<<GSHOTPyramid::DescriptorSize<<endl;
            model = Model();
            return is;
        }
		
		// Always set the offset and deformation of the root to zero
		if (!i) {
			parts[0].offset.setZero();
			parts[0].deformation.setZero();
		}
        // Always set the lvl offset of a part to zero
		else {
            parts[i].offset(3) = 0;
		}
		
        parts[i].filter() = GSHOTPyramid::Level(depths, rows, cols)().setConstant( GSHOTPyramid::Cell::Zero());
		
        for (int z = 0; z < depths; ++z) {
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    for (int j = 0; j < GSHOTPyramid::DescriptorSize; ++j)
                        is >> parts[i].filter()(z, y, x)(j);

                    // Always put the truncation feature at the end
                    if (nbFeatures < GSHOTPyramid::DescriptorSize)
                        swap(parts[i].filter()(z, y, x)(nbFeatures - 1),
                             parts[i].filter()(z, y, x)(GSHOTPyramid::DescriptorSize - 1));
                }
            }
		}
		
		if (!is) {
            cerr<<"Model::operator>> failed 3"<<endl;
			model = Model();
			return is;
		}
	}
	
	model = Model(parts, bias);
	
	return is;
}
