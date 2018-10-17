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

#include "Intersector.h"
#include "LBFGS.h"
#include "Mixture.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

using namespace Eigen;
using namespace FFLD;
using namespace std;

Mixture::Mixture() : cached_(false), zero_(true)
{
}

Mixture::Mixture(const vector<Model> & models) : models_(models), cached_(false), zero_(true)
{}

Mixture::Mixture(int nbComponents, const vector<Scene> & scenes, Object::Name name, int interval) :
cached_(false), zero_(true)
{
	// Create an empty mixture if any of the given parameters is invalid
	if ((nbComponents <= 0) || scenes.empty()) {
		cerr << "Attempting to create an empty mixture" << endl;
		return;
	}
	
	// Compute the root filters' sizes using Felzenszwalb's heuristic
    const vector<triple<int, int, int> > sizes = FilterSizes(nbComponents, scenes, name, interval);
	
	// Early return in case the root filters' sizes could not be determined
    if (sizes.size() != nbComponents){
        cerr<<"Root filters sizes could not be determined"<<endl;
        return;
    }
	
	// Initialize the models (with symmetry) to those sizes
    models_.resize(/*2 * */nbComponents);
	
	for (int i = 0; i < nbComponents; ++i) {
        models_[/*2 **/ i    ] = Model(sizes[i]);
        cout<<"Mixture:: model size set to : "<<sizes[i]<<endl;
//		models_[2 * i + 1] = Model(sizes[i]);
	}

}

bool Mixture::empty() const
{
	return models_.empty();
}

const vector<Model> & Mixture::models() const
{
	return models_;
}

vector<Model> & Mixture::models()
{
	return models_;
}

triple<int, int, int> Mixture::minSize() const
{
    triple<int, int, int> size(0, 0, 0);
	
	if (!models_.empty()) {
		size = models_[0].rootSize();
		
		for (int i = 1; i < models_.size(); ++i) {
			size.first = min(size.first, models_[i].rootSize().first);
			size.second = min(size.second, models_[i].rootSize().second);
            size.third = min(size.third, models_[i].rootSize().third);
		}
	}
	
	return size;
}

triple<int, int, int> Mixture::maxSize() const
{
    triple<int, int, int> size(0, 0, 0);
	
	if (!models_.empty()) {
		size = models_[0].rootSize();
		
		for (int i = 1; i < models_.size(); ++i) {
			size.first = max(size.first, models_[i].rootSize().first);
			size.second = max(size.second, models_[i].rootSize().second);
            size.third = max(size.third, models_[i].rootSize().third);
		}
	}
	
	return size;
}

double Mixture::train(const vector<Scene> & scenes, Object::Name name, Eigen::Vector3i pad,
					  int interval, int nbRelabel, int nbDatamine, int maxNegatives, double C,
					  double J, double overlap)
{
    if (empty() || scenes.empty() || (pad.x() < 1) || (pad.y() < 1) || (pad.z() < 1) || (interval < 1) ||
		(nbRelabel < 1) || (nbDatamine < 1) || (maxNegatives < models_.size()) || (C <= 0.0) ||
		(J <= 0.0) || (overlap <= 0.0) || (overlap >= 1.0)) {
		cerr << "Invalid training parameters" << endl;
		return numeric_limits<double>::quiet_NaN();
	}
	
	// Test if the models are really zero by looking at the first cell of the first filter of the
	// first model
	if (!models_[0].empty() && models_[0].parts()[0].filter.size() &&
        !models_[0].parts()[0].filter()(0, 0, 0).isZero()){
        zero_ = false;
    }

	double loss = numeric_limits<double>::infinity();

	for (int relabel = 0; relabel < nbRelabel; ++relabel) {
        cout<<"Mix::train relabel : "<< relabel <<endl;

		// Sample all the positives
		vector<pair<Model, int> > positives;

        posLatentSearch(scenes, name, pad, interval, overlap, positives);

        cout << "Mix::train found "<<positives.size() << " positives" << endl;

        // Cache of hard negative samples of maximum size maxNegatives
        vector<pair<Model, int> > negatives;
		
        // Previous loss on the cache
        double prevLoss = -numeric_limits<double>::infinity();
		
        for (int datamine = 0; datamine < nbDatamine; ++datamine) {
            // Remove easy samples (keep hard ones)
            int j = 0;
            cout<<"Mix::train datamine : "<<datamine<<endl;
            cout<<"Mix:: negatives.size1 : "<<negatives.size()<<" / j : "<<j<<endl;


            for (int i = 0; i < negatives.size(); ++i){
                cout<<"Mix::train old negatives["<<i<<"].score : "<<
                   negatives[i].first.parts()[0].deformation(4)<<endl;
                cout<<"Mix::train negatives["<<i<<"].first.parts()[0].deformation(4) : "<<
                   negatives[i].first.parts()[0].deformation(4)<<endl;
                negatives[i].first.parts()[0].deformation(4) =
                                     models_[negatives[i].second].dot(negatives[i].first);
                cout<<"Mix::train negatives["<<i<<"].first.parts()[0].deformation(4) : "<<
                   negatives[i].first.parts()[0].deformation(4)<<endl;
                if (negatives[i].first.parts()[0].deformation(4) > -1.1){
                    cout<<"Mix::train j : "<<j<<" / i :"<<i << endl;
                    negatives[j] = pair<Model, int>( negatives[i].first, negatives[i].second);
                    cout<<"Mix::train keep hard negative" << endl;
                    ++j;
                }
            }
            cout<<"Mix::train j : "<<j<<endl;

            negatives.resize(j);

            // Sample new hard negatives
            negLatentSearch(scenes, name, pad, interval, maxNegatives, negatives);

            cout<<"Mix:: negatives.size2 : "<<negatives.size()<<" / j : "<<j<<endl;
            //////
            // Stop if there are no new hard negatives
            if (datamine && (negatives.size() == j)){
                cout<<"Mix::train stop because no new hard negatives"<<endl;
                break;
            }


            const int maxIterations =
                min(max(10.0 * sqrt(static_cast<double>(positives.size())), 100.0), 1000.0);

            loss = trainSVM(positives, negatives, C, J, maxIterations);

            cout << "Relabel: " << relabel << ", datamine: " << datamine
                 << ", # positives: " << positives.size() << ", # hard negatives: " << j
                 << " (already in the cache) + " << (negatives.size() - j) << " (new) = "
                 << negatives.size() << ", loss (cache): " << loss << endl;


            // The filters definitely changed
            cached_ = false;
            zero_ = false;


            // Save the latest model so as to be able to look at it while training
            ofstream out("tmp.txt");
			
            out << (*this);
			
            // Stop if we are not making progress
            if ((0.999 * loss < prevLoss) && (negatives.size() < maxNegatives)){
                cout<<"Mix::train stop because not making progress"<<endl;
                break;
            }
			
            prevLoss = loss;
        }

	}
	
	return loss;
}

void Mixture::initializeParts(int nbParts/*, triple<int, int, int> partSize*/)
{
    for (int i = 0; i < models_.size(); ++i) {
        models_[i].initializeParts(nbParts, models_[i].rootSize());
	}
	
	// The filters definitely changed
	cached_ = false;
	zero_ = false;
}

void Mixture::computeScores(const GSHOTPyramid & pyramid, vector<vector<Tensor3DF> > & scores,
                                  vector<Indices> & argmaxes,
                                  vector<vector<vector<vector<Model::Positions> > > >* positions) const
{

    const int nbModels = static_cast<int>(models_.size());
    const int nbLevels = static_cast<int>(pyramid.levels().size());


    // Resize the scores and argmaxes
    scores.resize(nbLevels);
    for(int i=0;i<nbLevels;++i){
        scores[i].resize(pyramid.levels()[i].size());
    }
    argmaxes.resize(nbLevels);

    if (empty() || pyramid.empty()) {
        if(empty())
            cout << "computeEnergyScores::mixture models are empty" << empty() << endl;
        else
            cout << "computeEnergyScores::pyramid.empty()" << endl;
        scores.clear();
        argmaxes.clear();

        if (positions)
            positions->clear();

        return;
    }

    // Convolve with all the models
    vector<vector<vector<Tensor3DF> > > convolutions;//[model][lvl][box]
    convolve(pyramid, convolutions, positions);

    // In case of error
    if (convolutions.empty()) {
        cout << "Mix::computeScores::convolutions.empty()" << endl;
        scores.clear();
        argmaxes.clear();

//        if (positions)
//            positions->clear();
        cout << "Mix::computeScores::done" << endl;

        return;
    }


//#pragma omp parallel for
    for (int lvl = 0; lvl < nbLevels; ++lvl) {
        for (int box = 0; box < pyramid.levels()[lvl].size(); ++box) {

            int rows = static_cast<int>(convolutions[0][lvl][box].rows());
            int cols = static_cast<int>(convolutions[0][lvl][box].cols());
            int depths = static_cast<int>(convolutions[0][lvl][box].depths());


//            for (int i = 1; i < nbModels; ++i) {
//                rows = std::min(rows, static_cast<int>(convolutions[i][lvl][box].rows()));
//                cols = std::min(cols, static_cast<int>(convolutions[i][lvl][box].cols()));
//                depths = std::min(depths, static_cast<int>(convolutions[i][lvl][box].depths()));
//            }


            scores[lvl][box]().resize(depths, rows, cols);//convolution score of the best model at x, y, z
            argmaxes[lvl]().resize(depths, rows, cols);//indice of the best model at x, y, z

//            cout<<"ComputeScore:: score.depths() : "<< scores[lvl][box].depths() << endl;
//            cout<<"ComputeScore:: score.rows() : "<< scores[lvl][box].rows() << endl;
//            cout<<"ComputeScore:: score.cols() : "<< scores[lvl][box].cols() << endl;

            for (int z = 0; z < depths; ++z) {
                for (int y = 0; y < rows; ++y) {
                    for (int x = 0; x < cols; ++x) {
                        int argmax = 0;

                        for (int i = 1; i < nbModels; ++i){
                            if (convolutions[i][lvl][box]()(z, y, x) > convolutions[argmax][lvl][box]()(z, y, x))
                                argmax = i;
                        }

                        scores[lvl][box]()(z, y, x) = convolutions[argmax][lvl][box]()(z, y, x);
                        argmaxes[lvl]()(z, y, x) = argmax;
                    }
                }
            }
    //        cout << "Mix::computeScores scores["<<lvl<<"] is zero = "<<scores[lvl].isZero() << endl;
        }
    }
}

void Mixture::posLatentSearch(const vector<Scene> & scenes, Object::Name name, Eigen::Vector3i pad,
							  int interval, double overlap,
                              vector<pair<Model, int> > & positives) /*const*/
{
    cout << "Mix::posLatentSearch ..." << endl;
    if (scenes.empty() || (pad.x() < 1) || (pad.y() < 1) || (pad.z() < 1) || (interval < 1) || (overlap <= 0.0) ||
		(overlap >= 1.0)) {
		positives.clear();
		cerr << "Invalid training paramters" << endl;
		return;
	}
	
	positives.clear();
	
	for (int i = 0; i < scenes.size(); ++i) {
		// Skip negative scenes
		bool negative = true;
		
		for (int j = 0; j < scenes[i].objects().size(); ++j)
			if ((scenes[i].objects()[j].name() == name) && !scenes[i].objects()[j].difficult())
				negative = false;
		
		if (negative)
			continue;
		
        PointCloudPtr cloud( new PointCloudT);
		
        if( readPointCloud(scenes[i].filename(), cloud) == -1) {
            cout<<"couldnt open pcd file"<<endl;
			positives.clear();
			return;
		}
		
        int maxFilterSizes = max( models_[0].rootSize().first, max( models_[0].rootSize().second,
                models_[0].rootSize().third));
        triple<int, int, int> filterSizes(maxFilterSizes,maxFilterSizes,maxFilterSizes);

        cout << "Mix::posLatentSearch maxFilterSizes : " << maxFilterSizes << endl;

        const GSHOTPyramid pyramid(cloud, models_[0].rootSize(), interval, scenes[i].resolution());
		

//        cout << "Mix::posLatentSearch create pyramid of " << pyramid.levels().size() << " levels" << endl;


		if (pyramid.empty()) {
            cout<<"posLatentSearch::pyramid.empty"<<endl;
			positives.clear();
			return;
		}
		
        vector<vector<Tensor3DF> > scores;//[lvl][box]
        vector<Indices> argmaxes;//indices of model
        vector<vector<vector<vector<Model::Positions> > > >positions;//positions[nbModels][nbLvl][nbPart][box]
		
        if (!zero_){
            //only remaines score for the last octave
            computeScores(pyramid, scores, argmaxes, &positions);
        }


        // For each object, set as positive the best (highest score or else most intersecting)
        // position
        for (int j = 0; j < scenes[i].objects().size(); ++j) {
            // Ignore objects with a different name or difficult objects
            if ((scenes[i].objects()[j].name() != name) || scenes[i].objects()[j].difficult())
                continue;
			

//            cout<<"Mix::PosLatentSearch absolute positive box orig : "<<scenes[i].objects()[j].bndbox().getOriginCoordinate()<<endl;
//            cout<<"Mix::PosLatentSearch absolute positive box diago : "<<scenes[i].objects()[j].bndbox().getDiagonalCoordinate()<<endl;
//            cout<<"Mix::PosLatentSearch relative positive aabbox : "<<aabox<<endl;
            const Intersector intersector(scenes[i].objects()[j].bndbox(), overlap);

//            cout<<"Pos scenes[i].objects()[j].bndbox() : "<<scenes[i].objects()[j].bndbox()<<endl;

            // The model, level, position, score, and intersection of the best example
            int argModel = -1;
            int argBox = -1;
            int argX = -1;
            int argY = -1;
            int argZ = -1;
            int argLvl =-1;
            double maxScore = -numeric_limits<double>::infinity();
            double maxInter = 0.0;

			
            for (int lvl = 0; lvl < pyramid.levels().size(); ++lvl) {
                const double scale = 1 / pow(2.0, static_cast<double>(lvl) / interval);

                cout << "Mix::posLatentSearch lvl : " << lvl << endl;
                for (int box = 0; box < pyramid.levels()[lvl].size(); ++box) {

                    int rows = 0;
                    int cols = 0;
                    int depths = 0;

                    if (!zero_) {
                        depths = scores[lvl][box].depths();
                        rows = scores[lvl][box].rows();
                        cols = scores[lvl][box].cols();
                    }
                    else if (lvl >= interval) {
                        depths = pyramid.levels()[lvl][box].depths() - static_cast<int>(maxSize().first*scale) + 1;
                        rows = pyramid.levels()[lvl][box].rows() - static_cast<int>(maxSize().second*scale) + 1;
                        cols = pyramid.levels()[lvl][box].cols() - static_cast<int>(maxSize().third*scale)+ 1;
                    }

//                    cout << "Mix::posLatentSearch depths scene = " << depths << endl;
//                    cout << "Mix::posLatentSearch rows scene = " << rows << endl;
//                    cout << "Mix::posLatentSearch cols scene = " << cols << endl;

                    const PointCloudConstPtr boxCloud = pyramid.keypoints_[lvl][box];
                    PointType min;
                    PointType max;
                    pcl::getMinMax3D(*boxCloud, min, max);

                    if( abs(boxCloud->points[0].z-scenes[i].objects()[j].bndbox().origin()(0)) < pyramid.resolutions()[lvl] &&
                        abs(boxCloud->points[0].y-scenes[i].objects()[j].bndbox().origin()(1)) < pyramid.resolutions()[lvl] &&
                        abs(boxCloud->points[0].x-scenes[i].objects()[j].bndbox().origin()(2)) < pyramid.resolutions()[lvl]){
                        cout<<"Positif at box : "<<box<<endl;
                    }

                    if(depths*rows*cols > 0){

                                    // Find the best matching model (highest score or else most intersecting)
                        int model = 0;//zero_ ? 0 : argmaxes[lvl]()(z, y, x);
                        double intersection = -1.0;

                        // Try all models and keep the most intersecting one
                        if (zero_) {
                            for (int k = 0; k < models_.size(); ++k) {

                                Rectangle bndbox = pyramid.rectangles_[lvl][box];

    //                            cout<<"Pos:: bndbox : "<<bndbox<<endl;

                                double inter = 0.0;

                                if (intersector(bndbox, &inter)) {
                                    cout << "Mix::posLatentSearch intersector score : " << inter << " / " <<  intersection
                                         << " at box : " << box << endl;
                                    if (inter > intersection) {
    //                                            cout<<"Mix::PosLatentSearch try box orig : "<<bndbox.getOriginCoordinate()<<endl;
    //                                            cout<<"Mix::PosLatentSearch try box diago : "<<bndbox.getDiagonalCoordinate()<<endl;
                                        model = k;
                                        intersection = inter;
                                    }
                                } else{
    //                                        cout << "Mix::posLatentSearch wrong intersector score : " << inter << endl;
                                }
                            }
                        }
                        // Just take the model with the best score
                        else {
                            Rectangle bndbox = pyramid.rectangles_[lvl][box];

                            double inter = 0.0;

                            if(intersector(bndbox, &inter)){
                                cout << "Mix::posLatentSearch intersector score : " << inter << " / " <<  intersection
                                     << " at box : " << box << endl;
                                if (inter > intersection) {
                                    intersection = inter;
                                }
    //                                    cout << "Mix::posLatentSearch intersector True, scores = " << scores[lvl]()(z, y, x) <<" / "<< maxScore<< endl;
    //                                    cout << "Mix::posLatentSearch intersection = " << intersection <<" / "<< maxInter<< endl;
                            }
                        }


                        if ((intersection >= maxInter) /*&& (zero_ || (scores[lvl][box]()(0,0,0) > maxScore))*/) {
                            argModel = model;
                            argBox = box;
                            argX = 0;
                            argY = 0;
                            argZ = 0;
                            argLvl = lvl;

                            if (!zero_){
                                maxScore = scores[lvl][box]()(0,0,0);
                                cout << "Mix::posLatentSearch set maxScore = " << maxScore<< endl;
                            }

                            maxInter = intersection;
                        }
                    }

                }
            }
            cout << "maxInter : " << maxInter << " >= overlap :" << overlap << endl;

            if (maxInter >= overlap) {
                cout << "Mix:PosLatentSearch found a positive sample at : "
                     << argZ << " " << argY << " " << argX << " / " << pyramid.resolutions()[argLvl] << endl;
                cout << "Mix:PosLatentSearch found at : "
                     << pyramid.rectangles_[argLvl][argBox]
                     << " for box : " << argBox << endl;


                Model sample;
				
                models_[argModel].initializeSample(pyramid, argBox, argZ, argY, argX, argLvl, sample,
                                                   zero_ ? 0 : &positions[argModel]);
				
                if (!sample.empty())
                    positives.push_back(make_pair(sample, argModel));

            }
        }
    }
    cout << "pos latent search done" << endl;
}

static inline bool operator==(const Model & a, const Model & b)
{
	return (a.parts()[0].offset == b.parts()[0].offset) &&
		   (a.parts()[0].deformation(0) == b.parts()[0].deformation(0)) &&
           (a.parts()[0].deformation(1) == b.parts()[0].deformation(1) &&
           (a.parts()[0].deformation(2) == b.parts()[0].deformation(2)));
}

static inline bool operator<(const Model & a, const Model & b)
{
	return (a.parts()[0].offset(0) < b.parts()[0].offset(0)) ||
		   ((a.parts()[0].offset(0) == b.parts()[0].offset(0)) &&
			((a.parts()[0].offset(1) < b.parts()[0].offset(1)) ||
			 ((a.parts()[0].offset(1) == b.parts()[0].offset(1)) &&
			  ((a.parts()[0].deformation(0) < b.parts()[0].deformation(0)) ||
			   ((a.parts()[0].deformation(0) == b.parts()[0].deformation(0)) &&
                ((a.parts()[0].deformation(1) < b.parts()[0].deformation(1)) ||
                 ((a.parts()[0].deformation(1) == b.parts()[0].deformation(1)) &&
                  ((a.parts()[0].deformation(2) < b.parts()[2].deformation(1))))))))));
}

bool scoreComp( Vector4f a, Vector4f b){
    return a(3) > b(3);
}

void Mixture::negLatentSearch(const vector<Scene> & scenes, Object::Name name, Eigen::Vector3i pad,
                              int interval, int maxNegatives,
                              vector<pair<Model, int> > & negatives) const
{
    cout<<"Mix::negLatentSearch ..."<<endl;
    // Sample at most (maxNegatives - negatives.size()) negatives with a score above -1.0
    if (scenes.empty() || (pad.x() < 1) || (pad.y() < 1) || (pad.z() < 1) || (interval < 1) || (maxNegatives <= 0) ||
            (negatives.size() >= maxNegatives)) {
        negatives.clear();
        cerr << "NegLatentSearch::Invalid training paramters" << endl;
        return;
    }

    // The number of negatives already in the cache
    const int nbCached = static_cast<int>(negatives.size());

    for (int i = 0, j = 0; i < scenes.size(); ++i) {
        // Skip positive scenes
        bool positive = false;

        for (int k = 0; k < scenes[i].objects().size(); ++k)
            if (scenes[i].objects()[k].name() == name)
                positive = true;

        if (positive)
            continue;

        PointCloudPtr cloud( new PointCloudT);

        if (readPointCloud(scenes[i].filename(), cloud) == -1) {
            cout<<"Mix::negLatentSearch couldnt load PCD file"<<endl;
            negatives.clear();
            return;
        }


        int maxFilterSizes = max( models_[0].rootSize().first, max( models_[0].rootSize().second,
                models_[0].rootSize().third));
        triple<int, int, int> filterSizes(maxFilterSizes,maxFilterSizes,maxFilterSizes);

        const GSHOTPyramid pyramid(cloud, models_[0].rootSize(), interval, scenes[i].resolution());

        if (pyramid.empty()) {
            cout<<"Mix::negLatentSearch pyramid empty"<<endl;
            negatives.clear();
            return;
        }

        vector<vector<Tensor3DF> >scores;
        vector<Indices> argmaxes;
        vector<vector<vector<vector<Model::Positions> > > >positions;

        if (!zero_){
            computeScores(pyramid, scores, argmaxes, &positions);
        }


        for (int lvl = 0; lvl < pyramid.levels().size(); ++lvl) {
            const double scale = 1 / pow(2.0, static_cast<double>(lvl) / interval);

            for (int box = 0; box < pyramid.levels()[lvl].size(); ++box) {

                int rows = 0;
                int cols = 0;
                int depths = 0;

                if (!zero_) {
                    depths = scores[lvl][box].depths();
                    rows = scores[lvl][box].rows();
                    cols = scores[lvl][box].cols();

    //                // z,y,x,score
    //                vector<Vector4f> bestNeg( scores[lvl].size());
    //                for (int z = 0; z < depths; ++z) {
    //                    for (int y = 0; y < rows; ++y) {
    //                        for (int x = 0; x < cols; ++x) {
    //                            bestNeg[x+y*cols+z*cols*rows] = Vector4f( z, y, x, scores[lvl]()(z, y, x));
    //                        }
    //                    }
    //                }

    //                sort( bestNeg.begin(), bestNeg.end(), scoreComp);
    //                int i = 0;
    //                while( i < bestNeg.size()){
    //                    int z = bestNeg[i](0);
    //                    int y = bestNeg[i](1);
    //                    int x = bestNeg[i](2);
    //                    float score = bestNeg[i](3);
    //                    if(i<10) cout<<"NegLatentSearch:: score["<<i<<"] = "<<bestNeg[i]<<endl;

    //                    if( score > -1){

    //                        Model sample;
    //                        const int argmax = zero_ ? (rand() % models_.size()) : argmaxes[lvl]()(z, y, x);

    //                        models_[argmax].initializeSample(pyramid, z, y, x,
    //                                                         lvl, sample, zero_ ? 0 : &positions[argmax]);

    //                        if (!sample.empty()) {
    //                            // Store all the information about the sample in the offset and
    //                            // deformation of its root
    //                            sample.parts()[0].offset(0) = i;
    //                            sample.parts()[0].offset(1) = lvl;
    //                            sample.parts()[0].offset(2) = 0;
    //                            sample.parts()[0].offset(3) = 0;
    //                            sample.parts()[0].deformation(0) = z;
    //                            sample.parts()[0].deformation(1) = y;
    //                            sample.parts()[0].deformation(2) = x;
    //                            sample.parts()[0].deformation(3) = argmax;
    //                            sample.parts()[0].deformation(4) = zero_ ? 0.0 : score;
    //                            sample.parts()[0].deformation(5) = 0;
    //                            sample.parts()[0].deformation(6) = 0;
    //                            sample.parts()[0].deformation(7) = 0;
    //                        }
    //                        negatives.push_back(make_pair(sample, argmax));

    //                        if (negatives.size() == maxNegatives)
    //                            return;
    //                    }
    //                    ++i;
    //                }

                }
                else if (lvl >= interval) {
                    depths = static_cast<int>(pyramid.levels()[lvl][box].depths()) - maxSize().first*scale + 1;
                    rows = static_cast<int>(pyramid.levels()[lvl][box].rows()) - maxSize().second*scale + 1;
                    cols = static_cast<int>(pyramid.levels()[lvl][box].cols()) - maxSize().third*scale + 1;

    //                for (int z = 0; z < depths; ++z) {
    //                    for (int y = 0; y < rows; ++y) {
    //                        for (int x = 0; x < cols; ++x) {
    //                            Model sample;
    //                            const int argmax = zero_ ? (rand() % models_.size()) : argmaxes[lvl]()(z, y, x);

    //                            models_[argmax].initializeSample(pyramid, z, y, x,
    //                                                             lvl, sample, zero_ ? 0 : &positions[argmax]);

    //                            if (!sample.empty()) {
    //                                // Store all the information about the sample in the offset and
    //                                // deformation of its root
    //                                sample.parts()[0].offset(0) = i;
    //                                sample.parts()[0].offset(1) = lvl;
    //                                sample.parts()[0].offset(2) = 0;
    //                                sample.parts()[0].offset(3) = 0;
    //                                sample.parts()[0].deformation(0) = z;
    //                                sample.parts()[0].deformation(1) = y;
    //                                sample.parts()[0].deformation(2) = x;
    //                                sample.parts()[0].deformation(3) = argmax;
    //                                sample.parts()[0].deformation(4) = 0.0;
    //                                sample.parts()[0].deformation(5) = 0;
    //                                sample.parts()[0].deformation(6) = 0;
    //                                sample.parts()[0].deformation(7) = 0;
    //                            }
    //                            negatives.push_back(make_pair(sample, argmax));

    //                            if (negatives.size() == maxNegatives)
    //                                return;
    //                        }
    //                    }
    //                }
                }

                if(depths*rows*cols > 0){

                    if (!zero_) cout<<"Neg:: score = "<<scores[lvl][box]()(0,0,0)<<endl;

                    const int argmax = 0;/* zero_ ? (rand() % models_.size()) : argmaxes[lvl]()(z, y, x);*/

                    if (zero_ || (scores[lvl][box]()(0,0,0) > -1)) {
                        Model sample;

                        models_[argmax].initializeSample(pyramid, box, 0,0,0, lvl, sample,
                                                         zero_ ? 0 : &positions[argmax]);
                        if (!sample.empty()) {

                            // Store all the information about the sample in the offset and
                            // deformation of its root
                            sample.parts()[0].offset(0) = i;
                            sample.parts()[0].offset(1) = lvl;
                            sample.parts()[0].offset(2) = box;
                            sample.parts()[0].offset(3) = 0;
                            sample.parts()[0].deformation(0) = 0;
                            sample.parts()[0].deformation(1) = 0;
                            sample.parts()[0].deformation(2) = 0;
                            sample.parts()[0].deformation(3) = argmax;
                            sample.parts()[0].deformation(4) = zero_ ? 0.0 : scores[lvl][box]()(0,0,0);
                            sample.parts()[0].deformation(5) = 0;
                            sample.parts()[0].deformation(6) = 0;
                            sample.parts()[0].deformation(7) = 0;


                            // Look if the same sample was already sampled
                            while ((j < nbCached) && (negatives[j].first < sample))
                                ++j;

                            if (!zero_) cout<<"Neg:: j = "<<j<<endl;
                            // Make sure not to put the same sample twice
                            if ((j >= nbCached) || !(negatives[j].first == sample)) {
                                cout<<"Mix::negLatentSearch add new sample with score "
                                   <<sample.parts()[0].deformation(4)<<endl;

                                negatives.push_back(make_pair(sample, argmax));

                                if (negatives.size() == maxNegatives)
                                    return;
                            }
                        }
                    }
                }
            }
        }
    }
}

namespace FFLD
{
namespace detail
{
class Loss : public LBFGS::IFunction
{
public:
	Loss(vector<Model> & models, const vector<pair<Model, int> > & positives,
		 const vector<pair<Model, int> > & negatives, double C, double J, int maxIterations) :
	models_(models), positives_(positives), negatives_(negatives), C_(C), J_(J),
	maxIterations_(maxIterations)
	{
	}
	
	virtual int dim() const
	{
		int d = 0;
		
		for (int i = 0; i < models_.size(); ++i) {
			for (int j = 0; j < models_[i].parts().size(); ++j) {
                d += models_[i].parts()[j].filter.size() * GSHOTPyramid::DescriptorSize; // Filter
				
				if (j)
                    d += 8; // Deformation
			}
			
			++d; // Bias
		}
		
		return d;
	}
	
	virtual double operator()(const double * x, double * g = 0) const
	{
		// Recopy the features into the models
		ToModels(x, models_);
		
		// Compute the loss and gradient over the samples
		double loss = 0.0;
		
		vector<Model> gradients;
		
		if (g) {
			gradients.resize(models_.size());
			
			for (int i = 0; i < models_.size(); ++i)
				gradients[i] = Model(models_[i].rootSize(),
									 static_cast<int>(models_[i].parts().size()) - 1,
									 models_[i].partSize());
		}


        vector<double> posMargins(positives_.size());
		
//#pragma omp parallel for
		for (int i = 0; i < positives_.size(); ++i)
			posMargins[i] = models_[positives_[i].second].dot(positives_[i].first);
		
// Never use #pragma omp parallel for HERE
		for (int i = 0; i < positives_.size(); ++i) {
			if (posMargins[i] < 1.0) {
				loss += 1.0 - posMargins[i];
				
				if (g)
					gradients[positives_[i].second] -= positives_[i].first;
            }
		}
		

        // Reweight thpositives
		if (J_ != 1.0) {
			loss *= J_;
			
			if (g) {
				for (int i = 0; i < models_.size(); ++i)
					gradients[i] *= J_;
			}
		}

		vector<double> negMargins(negatives_.size());
		
//#pragma omp parallel for
		for (int i = 0; i < negatives_.size(); ++i)
			negMargins[i] = models_[negatives_[i].second].dot(negatives_[i].first);
		
//#pragma omp parallel for
		for (int i = 0; i < negatives_.size(); ++i) {
			if (negMargins[i] > -1.0) {
				loss += 1.0 + negMargins[i];
				
				if (g)
					gradients[negatives_[i].second] += negatives_[i].first;
			}
		}

		// Add the loss and gradient of the regularization term
		double maxNorm = 0.0;
		int argNorm = 0;
		
		for (int i = 0; i < models_.size(); ++i) {
			if (g)
				gradients[i] *= C_;

			const double norm = models_[i].norm();

			if (norm > maxNorm) {
				maxNorm = norm;
				argNorm = i;
			}
		}

		// Recopy the gradient if needed
		if (g) {
			// Regularization gradient
			gradients[argNorm] += models_[argNorm];
			
			// Regularize the deformation 10 times more
			for (int i = 1; i < gradients[argNorm].parts().size(); ++i)
				gradients[argNorm].parts()[i].deformation +=
					9.0 * models_[argNorm].parts()[i].deformation;
			
			// Do not regularize the bias
			gradients[argNorm].bias() -= models_[argNorm].bias();

			// In case minimum constraints were applied
			for (int i = 0; i < models_.size(); ++i) {
				for (int j = 1; j < models_[i].parts().size(); ++j) {
					if (models_[i].parts()[j].deformation(0) >= -0.005)
						gradients[i].parts()[j].deformation(0) =
							max(gradients[i].parts()[j].deformation(0), 0.0);
					
					if (models_[i].parts()[j].deformation(2) >= -0.005)
						gradients[i].parts()[j].deformation(2) =
							max(gradients[i].parts()[j].deformation(2), 0.0);
					
					if (models_[i].parts()[j].deformation(4) >= -0.005)
						gradients[i].parts()[j].deformation(4) =
							max(gradients[i].parts()[j].deformation(4), 0.0);

                    if (models_[i].parts()[j].deformation(6) >= -0.005)
                        gradients[i].parts()[j].deformation(6) =
                            max(gradients[i].parts()[j].deformation(6), 0.0);
				}
			}

			FromModels(gradients, g);
		}
		
		return 0.5 * maxNorm * maxNorm + C_ * loss;
	}
	
	static void ToModels(const double * x, vector<Model> & models)
	{
		for (int i = 0, j = 0; i < models.size(); ++i) {
			for (int k = 0; k < models[i].parts().size(); ++k) {
				const int nbFeatures = static_cast<int>(models[i].parts()[k].filter.size()) *
                                       GSHOTPyramid::DescriptorSize;
				
                copy(x + j, x + j + nbFeatures, models[i].parts()[k].filter().data()->data());
				
				j += nbFeatures;
				
				if (k) {
					// Apply minimum constraints
                    models[i].parts()[k].deformation(0) = std::min((x + j)[0],-0.005);
					models[i].parts()[k].deformation(1) = (x + j)[1];
                    models[i].parts()[k].deformation(2) = std::min((x + j)[2],-0.005);
					models[i].parts()[k].deformation(3) = (x + j)[3];
                    models[i].parts()[k].deformation(4) = std::min((x + j)[4],-0.005);
					models[i].parts()[k].deformation(5) = (x + j)[5];
                    models[i].parts()[k].deformation(6) = std::min((x + j)[6],-0.005);
                    models[i].parts()[k].deformation(7) = (x + j)[7];
					
                    j += 8;
				}
			}
			
			models[i].bias() = x[j];
			
			++j;
		}
	}
	
	static void FromModels(const vector<Model> & models, double * x)
	{
		for (int i = 0, j = 0; i < models.size(); ++i) {
			for (int k = 0; k < models[i].parts().size(); ++k) {
				const int nbFeatures = static_cast<int>(models[i].parts()[k].filter.size()) *
                                       GSHOTPyramid::DescriptorSize;
				
                copy(models[i].parts()[k].filter().data()->data(),
                     models[i].parts()[k].filter().data()->data() + nbFeatures, x + j);
				
				j += nbFeatures;
				
				if (k) {
					copy(models[i].parts()[k].deformation.data(),
                         models[i].parts()[k].deformation.data() + 8, x + j);
					
                    j += 8;
				}
			}
			
			x[j] = models[i].bias();
			++j;
		}
	}
	
private:
	vector<Model> & models_;
	const vector<pair<Model, int> > & positives_;
	const vector<pair<Model, int> > & negatives_;
	double C_;
	double J_;
	int maxIterations_;
};}
}

double Mixture::trainSVM(const vector<pair<Model, int> > & positives,
					  const vector<pair<Model, int> > & negatives, double C, double J,
					  int maxIterations)
{

	detail::Loss loss(models_, positives, negatives, C, J, maxIterations);

    double epsilon = 0.001;
    LBFGS lbfgs(&loss, epsilon, maxIterations, 20, 20);

	
	// Start from the current models
	VectorXd x(loss.dim());
	
	detail::Loss::FromModels(models_, x.data());

	const double l = lbfgs(x.data());

	detail::Loss::ToModels(x.data(), models_);

	return l;
}

void Mixture::convolve(const GSHOTPyramid & pyramid,
                       vector<vector<vector<Tensor3DF> > > & scores,//[model][lvl][box]
                       vector<vector<vector<vector<Model::Positions> > > > * positions) const//[model.size][model.part.size][pyramid.lvl.size][box]
{

	if (empty() || pyramid.empty()) {
        if (empty()) cout<<"Mix::convolve mixture is empty"<<endl;
        else cout<<"Mix::convolve pyramid is empty"<<endl;

		scores.clear();
		
		if (positions)
			positions->clear();
		
		return;
	}
	
	const int nbModels = static_cast<int>(models_.size());
	
	// Resize the scores and positions
	scores.resize(nbModels);
	
	if (positions)
		positions->resize(nbModels);
	
	// Transform the filters if needed

//#pragma omp parallel for
    for (int i = 0; i < nbModels; ++i){
        models_[i].convolve(pyramid, scores[i], positions ? &(*positions)[i] : 0);
    }

}

std::vector<triple<int, int, int> > Mixture::FilterSizes(int nbComponents, const vector<Scene> & scenes,
                                             Object::Name name, int interval)
{
	// Early return in case the filters or the dataset are empty
    if ((nbComponents <= 0) || scenes.empty()){
        cerr<<"nbComponents <= 0 or scenes.empty()"<<endl;
        return std::vector<triple<int, int, int> >();
    }

    const float scale = 1 / pow(2.0, interval);
	// Sort the aspect ratio of all the (non difficult) samples
    vector<Rectangle> rects;
	
	for (int i = 0; i < scenes.size(); ++i) {
		for (int j = 0; j < scenes[i].objects().size(); ++j) {
			const Object & obj = scenes[i].objects()[j];
			
            if ((obj.name() == name) && !obj.difficult()){
                rects.push_back( obj.bndbox());
                cout<<"Mixture::FilterSizes rects : "<<obj.bndbox()<<endl;
            }
		}
	}
	
	// Early return if there is no object
    if (rects.empty()){
        cerr<<"rects.empty()"<<endl;
        return std::vector<triple<int, int, int> >();
    }
	
	// Sort the aspect ratio of all the samples
    sort(rects.begin(), rects.end());
//    reverse(rects.begin(), rects.end());
	
	// For each mixture model
    vector<int> references(nbComponents+1);
    std::vector<triple<int, int, int> > sizes(nbComponents);

    for (int i = 0; i <= nbComponents; ++i){
        references[i] = rects.size() * i / nbComponents;
    }

    for (int i = 0; i < references.size(); ++i){
        cout<<"Mixture::FilterSizes ref : "<<references[i]<<endl;
    }
	
    for (int i = 0; i < nbComponents; ++i) {
        Vector3f boxSize(0,0,0);

        for (int j = references[i]; j < references[i+1]; ++j) {
            boxSize += rects[j].size();
        }

        cout<<"Mixture::FilterSizes boxSize : "<<endl<<boxSize<<endl;

        sizes[i].first = ceil( boxSize(0) / (references[i+1]-references[i]) * scale / scenes[i].resolution());
        sizes[i].second = ceil( boxSize(1) / (references[i+1]-references[i]) * scale / scenes[i].resolution());
        sizes[i].third = ceil( boxSize(2) / (references[i+1]-references[i]) * scale / scenes[i].resolution());
    }
	
	return sizes;
}

ostream & FFLD::operator<<(ostream & os, const Mixture & mixture)
{
	// Save the number of models (mixture components)
	os << mixture.models().size() << endl;
	
	// Save the models themselves
	for (int i = 0; i < mixture.models().size(); ++i)
		os << mixture.models()[i] << endl;
	
	return os;
}

istream & FFLD::operator>>(istream & is, Mixture & mixture)
{
	int nbModels;
	
	is >> nbModels;
	
	if (!is || (nbModels <= 0)) {
        cerr<<"Mixture::operator>> failed 1"<<endl;
		mixture = Mixture();
		return is;
	}
	
	vector<Model> models(nbModels);
	
	for (int i = 0; i < nbModels; ++i) {
		is >> models[i];
		
		if (!is || models[i].empty()) {
            cerr<<"Mixture::operator>> failed 2"<<endl;
			mixture = Mixture();
			return is;
		}
	}
    mixture.models().swap(models);
	
	return is;
}
