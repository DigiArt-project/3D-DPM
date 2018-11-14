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
    const vector<Vector3i> sizes = {Vector3i(1,1,1)};//FilterSizes(nbComponents, scenes, name, interval);//{Vector3i(6,3,5)};//{Vector3i(2,3,2)*2};
	

//	// Early return in case the root filters' sizes could not be determined
//    if (sizes.size() != nbComponents){
//        cerr<<"Root filters sizes could not be determined"<<endl;
//        return;
//    }
	
	// Initialize the models (with symmetry) to those sizes
    models_.resize( nbComponents);
	
	for (int i = 0; i < nbComponents; ++i) {
        models_[i] = Model(sizes[i]);
        cout<<"Mixture:: model size set to : "<<sizes[i]<<endl;        
	}

    models_[0].boxSize_ = FilterSizes(nbComponents, scenes, name, interval);

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

Vector3i Mixture::minSize() const
{
    Vector3i size(0, 0, 0);
	
	if (!models_.empty()) {
		size = models_[0].rootSize();
		
		for (int i = 1; i < models_.size(); ++i) {
            size(0) = min(size(0), models_[i].rootSize()(0));
            size(1) = min(size(1), models_[i].rootSize()(1));
            size(2) = min(size(2), models_[i].rootSize()(2));
		}
	}
	
	return size;
}

Vector3i Mixture::maxSize() const
{
    Vector3i size(0, 0, 0);
	
	if (!models_.empty()) {
		size = models_[0].rootSize();
		
		for (int i = 1; i < models_.size(); ++i) {
            size(0) = max(size(0), models_[i].rootSize()(0));
            size(1) = max(size(1), models_[i].rootSize()(1));
            size(2) = max(size(2), models_[i].rootSize()(2));
		}
	}
	
	return size;
}

double Mixture::train(const vector<Scene> & scenes, Object::Name name, int nbParts,
					  int interval, int nbRelabel, int nbDatamine, int maxNegatives, double C,
                      double J, double overlap, float negOverlap)
{
    if (empty() || scenes.empty() || (interval < 1) ||
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
        vector<GSHOTPyramid::Level> positiveParts;

        posLatentSearch(scenes, name, interval, overlap, positives, positiveParts);


        cout << "Mix::train found "<<positives.size() << " positives" << endl;

        // Cache of hard negative samples of maximum size maxNegatives
        vector<pair<Model, int> > negatives;
		
        // Previous loss on the cache
        double prevLoss = -numeric_limits<double>::infinity();
		
        for (int datamine = 0; datamine < nbDatamine; ++datamine) {
            // Remove easy samples (keep hard ones)
            int j = 0;
            cout<<"Mix::train datamine : "<<datamine<<endl;
//            cout<<"Mix:: negatives.size1 : "<<negatives.size()<<" / j : "<<j<<endl;


            for (int i = 0; i < negatives.size(); ++i){
//                cout<<"Mix::train old negatives["<<i<<"].score : "<<
//                   negatives[i].first.parts()[0].deformation(4)<<endl;
//                cout<<"Mix::train negatives["<<i<<"].first.parts()[0].deformation(4) : "<<
//                   negatives[i].first.parts()[0].deformation(4)<<endl;
                negatives[i].first.parts()[0].deformation(7) =
                                     models_[negatives[i].second].dot(negatives[i].first);

                if (negatives[i].first.parts()[0].deformation(7) > -1){
                    cout<<"Mix::train keep negatives["<<i<<"].first.parts()[0].deformation(4) : "<<
                       negatives[i].first.parts()[0].deformation(7)<<endl;
//                    cout<<"Mix::train j : "<<j<<" / i :"<<i << endl;
                    negatives[j] = pair<Model, int>( negatives[i].first, negatives[i].second);
//                    cout<<"Mix::train keep hard negative" << endl;
                    ++j;
                }
            }
//            cout<<"Mix::train j : "<<j<<endl;

            negatives.resize(j);

            // Sample new hard negatives
            negLatentSearch(scenes, name, interval, maxNegatives, negOverlap, negatives);

            cout<<"Mix:: negatives.size2 : "<<negatives.size()<<" / j : "<<j<<endl;
            //////
            // Stop if there are no new hard negatives
            if (datamine && (negatives.size() == j || j>=maxNegatives) || negatives.size() == 0){
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

            cout<< "model score vs positive : "<<positives[0].first.parts()[0].filter.dot(
                        models_[0].parts()[0].filter)<<endl;



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
        ///initParts
        if(zero_ && positiveParts.size()){
            GSHOTPyramid::Level meanParts = positiveParts[0];
            for(int i = 1; i < positiveParts.size();++i){
                meanParts += positiveParts[i];
            }
            initializeParts(nbParts, meanParts);
    //                cout<<"parts size : "<<models_[0].parts()
        }
        ///

	}

    // The filters definitely changed
    cached_ = false;
    zero_ = false;
	return loss;
}

void Mixture::initializeParts(int nbParts, GSHOTPyramid::Level parts)
{
    for (int i = 0; i < models_.size(); ++i) {
        if( nbParts){
//            Vector3i partSize(models_[i].rootSize()(0)*2*0.9283/pow(nbParts, 0.33),
//                              models_[i].rootSize()(1)*2*0.9283/pow(nbParts, 0.33),
//                              models_[i].rootSize()(2)*2*0.9283/pow(nbParts, 0.33));
            Vector3i partSize(1,1,1);
            cout<<"initParts : "<<partSize<<endl;
            models_[i].initializeParts(nbParts, partSize, parts);
        }
	}
	
	// The filters definitely changed
	cached_ = false;
	zero_ = false;
}

void Mixture::computeScores(const GSHOTPyramid & pyramid, vector<vector<Tensor3DF> > & scores,
                                  vector<Indices> & argmaxes,
                                  vector<vector<vector<vector<Model::Positions> > > >* positions) const
{
    cout << "computeScores::start" << endl;

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

    cout << "computeScores::middle" << endl;

#pragma omp parallel for
    for (int lvl = 0; lvl < nbLevels; ++lvl) {
#pragma omp parallel for
        for (int box = 0; box < pyramid.levels()[lvl].size(); ++box) {

            int rows = static_cast<int>(convolutions[0][lvl][box].rows());
            int cols = static_cast<int>(convolutions[0][lvl][box].cols());
            int depths = static_cast<int>(convolutions[0][lvl][box].depths());


//            for (int i = 1; i < nbModels; ++i) {
//                rows = std::min(rows, static_cast<int>(convolutions[i][lvl][box].rows()));
//                cols = std::min(cols, static_cast<int>(convolutions[i][lvl][box].cols()));
//                depths = std::min(depths, static_cast<int>(convolutions[i][lvl][box].depths()));
//            }


            scores[lvl][box]().resize(Eigen::array<long int, 3>{{depths, rows, cols}});//convolution score of the best model at x, y, z
            argmaxes[lvl]().resize(Eigen::array<long int, 3>{{depths, rows, cols}});//indice of the best model at x, y, z

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
    cout << "computeScores::end" << endl;

}

vector<Rectangle> Mixture::posLatentSearch(const vector<Scene> & scenes, Object::Name name,
							  int interval, double overlap,
                              vector<pair<Model, int> > & positives,
                              vector<GSHOTPyramid::Level> & positivesParts) /*const*/
{
    vector<Rectangle> recs;

    cout << "Mix::posLatentSearch ..." << endl;
    if (scenes.empty() || (interval < 1) || (overlap <= 0.0) ||
		(overlap >= 1.0)) {
		positives.clear();
		cerr << "Invalid training paramters" << endl;
        return recs;
	}

	
	positives.clear();
	
	for (int i = 0; i < scenes.size(); ++i) {
		// Skip negative scenes
        vector<Vector3i> colors;
		
        for (int j = 0; j < scenes[i].objects().size(); ++j){
            if ( scenes[i].objects()[j].name() == name){
                colors.push_back(scenes[i].objects()[j].color());
            }
        }
		
        if (!colors.empty()){
		
            PointCloudPtr cloud( new PointCloudT);

            if( readPointCloud(scenes[i].filename(), cloud) == -1) {
                cout<<"couldnt open pcd file"<<endl;
                positives.clear();
                return recs;
            }

            PointCloudPtr finalCloud (new PointCloudT( 0,1,PointType()));
            cout << "Mix::posLatentSearch finalCloud.size : " << finalCloud->size() << endl;

            if (!zero_){
                for(int j = 0; j < colors.size(); ++j){
                    const Rectangle& rec = scenes[i].objects()[j].bndbox();
                    Vector4f ptStart( rec.origin(0)-rec.size(0)*(1-overlap),
                                      rec.origin(1)-rec.size(1)*(1-overlap),
                                      rec.origin(2)-rec.size(2)*(1-overlap), 1);
                    Vector4f ptEnd( rec.origin(0)+(2-overlap)*rec.size(0),
                                    rec.origin(1)+(2-overlap)*rec.size(1),
                                    rec.origin(2)+(2-overlap)*rec.size(2), 1);
                    std::vector<int> pt_indices;
                    pcl::getPointsInBox(*cloud, ptStart, ptEnd, pt_indices);

                    int tmp = finalCloud->points.size();
                    finalCloud->width    = tmp+pt_indices.size();
                    finalCloud->height   = 1;
                    finalCloud->points.resize (finalCloud->width);
                    #pragma omp parallel for
                    for(int k = 0; k < pt_indices.size(); ++k){
                        finalCloud->points[tmp+k] = cloud->points[pt_indices[k]];
                    }
                }

            }else{
                for(int k = 0; k < cloud->size(); ++k){
                    for(int j = 0; j < colors.size(); ++j){
                        if( cloud->points[k].getRGBVector3i() == colors[j])
                        {
                            finalCloud->width    = finalCloud->points.size()+1;
                            finalCloud->height   = 1;
                            finalCloud->points.resize (finalCloud->width);
                            finalCloud->at(finalCloud->points.size()-1) = cloud->points[k];
                        }
                    }
                }
            }


            Vector3i rootSize(2,3,2);
            GSHOTPyramid pyramid(models()[0].boxSize_, models_[0].parts().size(), interval, scenes[i].resolution());


            cout << "Mix::posLatentSearch finalCloud.size2 : " << finalCloud->size() << endl;

            PointType minTmp;
            PointType min;
            PointType max;
            pcl::getMinMax3D(*cloud, minTmp, max);

            min.x = floor(minTmp.x/scenes[i].resolution())*scenes[i].resolution();
            min.y = floor(minTmp.y/scenes[i].resolution())*scenes[i].resolution();
            min.z = floor(minTmp.z/scenes[i].resolution())*scenes[i].resolution();


            vector<vector<Tensor3DF> > scores;//[lvl][box]
            vector<Indices> argmaxes;//indices of model
            vector<vector<vector<vector<Model::Positions> > > >positions;//positions[nbModels][nbLvl][nbPart][box]

            if (!zero_){
//                pyramid.createFilteredPyramid(finalCloud, models_[0].parts()[0].filter,
//                        min, max, 0, 20);
                pyramid.createFullPyramid(finalCloud, min, max, 5);

                //only remaines score for the last octave
                computeScores(pyramid, scores, argmaxes, &positions);
            }else{
                pyramid.createFullPyramid(finalCloud, min, max, 5);
            }



            if (pyramid.empty()) {
                cout<<"posLatentSearch::pyramid.empty"<<endl;
                positives.clear();
                return recs;
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
                const Intersector intersector(scenes[i].objects()[j].bndbox().cloud(),
                                              scenes[i].objects()[j].bndbox().volume(), overlap);

                cout<<"Pos scenes["<<i<<"].objects()[j].bndbox() : "<<scenes[i].objects()[j].bndbox()<<endl;

                // The model, level, position, score, and intersection of the best example
                int argModel = -1;
                int argBox = -1;
                int argX = -1;
                int argY = -1;
                int argZ = -1;
                int argLvl =-1;
                double maxScore = -numeric_limits<double>::infinity();
                double maxInter = 0.0;

//                #pragma omp parallel for
                for (int lvl = 0; lvl < pyramid.levels().size(); ++lvl) {
                    const double scale = 1 / pow(2.0, static_cast<double>(lvl) / interval);

                    cout << "Mix::posLatentSearch lvl : " << lvl << endl;
//                    #pragma omp parallel for
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
                            depths = pyramid.levels()[lvl][box].depths() - static_cast<int>(maxSize()(0)*scale) + 1;
                            rows = pyramid.levels()[lvl][box].rows() - static_cast<int>(maxSize()(1)*scale) + 1;
                            cols = pyramid.levels()[lvl][box].cols() - static_cast<int>(maxSize()(2)*scale)+ 1;
                        }

    //                    cout << "Mix::posLatentSearch depths scene = " << depths << endl;
    //                    cout << "Mix::posLatentSearch rows scene = " << rows << endl;
    //                    cout << "Mix::posLatentSearch cols scene = " << cols << endl;

                        const PointCloudConstPtr boxCloud = pyramid.keyPts_[lvl][box];
                        PointType min;
                        PointType max;
                        pcl::getMinMax3D(*boxCloud, min, max);

//                        if( abs(boxCloud->points[0].z-scenes[i].objects()[j].bndbox().origin()(0)) < pyramid.resolutions()[lvl] &&
//                            abs(boxCloud->points[0].y-scenes[i].objects()[j].bndbox().origin()(1)) < pyramid.resolutions()[lvl] &&
//                            abs(boxCloud->points[0].x-scenes[i].objects()[j].bndbox().origin()(2)) < pyramid.resolutions()[lvl]){
//                            cout<<"Positif at box : "<<box<<endl;
//                        }

                        if(depths*rows*cols > 0){

                                        // Find the best matching model (highest score or else most intersecting)
                            int model = 0;//zero_ ? 0 : argmaxes[lvl]()(z, y, x);
                            double intersection = -1.0;

                            // Try all models and keep the most intersecting one
                            if (zero_) {
                                for (int k = 0; k < models_.size(); ++k) {

                                    Rectangle bndbox = pyramid.rectangles_[lvl][box];
//                                    cout << "Mix::posLatentSearch bbox : " << bndbox << endl;

                                    double inter = 0.0;

                                    if (intersector(bndbox.cloud(), bndbox.volume(), &inter)) {
    //                                    cout << "Mix::posLatentSearch intersector score : " << inter << " / " <<  intersection
    //                                         << " at box : " << box << endl;
    //                                    cout << "Mix::posLatentSearch bbox : " << bndbox << endl;
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

//                                cout << "Mix::posLatentSearch bbox : " << bndbox << endl;
                                double inter = 0.0;

                                if(intersector(bndbox.cloud(), bndbox.volume(), &inter)){
    //                                cout << "Mix::posLatentSearch intersector score : " << inter << " / " <<  intersection
    //                                     << " at box : " << box << endl;
    //                                cout << "Mix::posLatentSearch bbox : " << bndbox << endl;
                                    if (inter > intersection) {
                                        intersection = inter;
                                    }
        //                                    cout << "Mix::posLatentSearch intersector True, scores = " << scores[lvl]()(z, y, x) <<" / "<< maxScore<< endl;
        //                                    cout << "Mix::posLatentSearch intersection = " << intersection <<" / "<< maxInter<< endl;
                                }
                            }
//                            if ((intersection >= overlap && zero_) ||
//                                    (!zero_ && scores[lvl][box]()(0,0,0) > maxScore && intersection >= overlap)) {
                            if ((intersection >= maxInter )) {
//                            if ((intersection >= maxInter) && (zero_ || (scores[lvl][box]()(0,0,0) > maxScore))) {
                                argModel = model;
                                argBox = box;
                                argX = 0;
                                argY = 0;
                                argZ = 0;
                                argLvl = lvl;

                                if (!zero_){
                                    maxScore = scores[lvl][box]()(0,0,0);
    //                                cout << "Mix::posLatentSearch set maxScore = " << maxScore<< endl;
                                }

                                maxInter = intersection;

                                cout << "Mix:PosLatentSearch found at : "
                                     << pyramid.rectangles_[argLvl][argBox]
                                     << " for box : " << argBox << endl;

                                Model sample;

//                                cout << "Mix::posLatentSearch rf of positive sample : ";
//                                for(int i=0;i<9;++i){
//                                    cout<<pyramid.globalDescriptors->points[argBox].rf[i]<<" ";
//                                }
//                                cout<<endl;

                                models_[argModel].initializeSample(pyramid, argBox, argZ, argY, argX, argLvl, sample,
                                                                   zero_ ? 0 : &positions[argModel]);

                                if (!sample.empty()){
                                    positives.push_back(make_pair(sample, argModel));
                                    recs.push_back(pyramid.rectangles_[argLvl][argBox]);
                                    if(zero_){
                                        positivesParts.push_back(pyramid.levels()[0][argBox]);
                                    }
                                }


                            }
                        }

                    }
                }
                cout << "maxInter : " << maxInter << " >= overlap :" << overlap << endl;

                if (maxInter >= overlap) {
    //                cout << "Mix:PosLatentSearch found a positive sample at : "
    //                     << argZ << " " << argY << " " << argX << " / " << pyramid.resolutions()[argLvl] << endl;
//                    cout << "Mix:PosLatentSearch found at : "
//                         << pyramid.rectangles_[argLvl][argBox]
//                         << " for box : " << argBox << endl;


//                    Model sample;

//                    cout << "Mix::posLatentSearch rf of positive sample : ";
//                    for(int i=0;i<9;++i){
//                        cout<<pyramid.globalDescriptors->points[argBox].rf[i]<<" ";
//                    }
//                    cout<<endl;

//                    models_[argModel].initializeSample(pyramid, argBox, argZ, argY, argX, argLvl, sample,
//                                                       zero_ ? 0 : &positions[argModel]);

//                    if (!sample.empty()){
//                        positives.push_back(make_pair(sample, argModel));
//                        recs.push_back(pyramid.rectangles_[argLvl][argBox]);
//                        if(zero_){
//                            positivesParts.push_back(pyramid.levels()[0][argBox]);
//                        }
//                    }

                }
            }
        }
    }
    cout << "pos latent search done" << endl;
    return recs;
}

static inline bool operator==(const Model & a, const Model & b)
{
    return (a.parts()[0].offset == b.parts()[0].offset)/* &&
		   (a.parts()[0].deformation(0) == b.parts()[0].deformation(0)) &&
           (a.parts()[0].deformation(1) == b.parts()[0].deformation(1) &&
           (a.parts()[0].deformation(2) == b.parts()[0].deformation(2)))*/;
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

void Mixture::negLatentSearch(const vector<Scene> & scenes, Object::Name name,
                              int interval, int maxNegatives, float overlap,
                              vector<pair<Model, int> > & negatives) const
{
    cout<<"Mix::negLatentSearch ..."<<endl;
    // Sample at most (maxNegatives - negatives.size()) negatives with a score above -1.0
    if (scenes.empty() || (interval < 1) || (maxNegatives <= 0) ||
            (negatives.size() >= maxNegatives)) {
        negatives.clear();
        cerr << "NegLatentSearch::Invalid training paramters" << endl;
        return;
    }

    // The number of negatives already in the cache
    const int nbCached = static_cast<int>(negatives.size());

    for (int i = 0; i < scenes.size(); ++i) {
        // Skip positive scenes
//        bool positive = false;

//        for (int k = 0; k < scenes[i].objects().size(); ++k)
//            if (scenes[i].objects()[k].name() == name)
//                positive = true;

//        if (positive)
//            continue;

        vector<Intersector> intersectors;
        for (int k = 0; k < scenes[i].objects().size(); ++k){
            if (scenes[i].objects()[k].name() == name){
                cout<<"neg sample with positive ..."<<endl;
                intersectors.push_back(Intersector(scenes[i].objects()[k].bndbox().cloud(),
                                                   scenes[i].objects()[k].bndbox().volume(), overlap));
            }
        }

        PointCloudPtr cloud( new PointCloudT);

        if (readPointCloud(scenes[i].filename(), cloud) == -1) {
            cout<<"Mix::negLatentSearch couldnt load PCD file"<<endl;
            negatives.clear();
            return;
        }


        PointType minTmp;
        PointType min;
        PointType max;
        pcl::getMinMax3D(*cloud, minTmp, max);

        min.x = floor(minTmp.x/scenes[i].resolution())*scenes[i].resolution();
        min.y = floor(minTmp.y/scenes[i].resolution())*scenes[i].resolution();
        min.z = floor(minTmp.z/scenes[i].resolution())*scenes[i].resolution();

        GSHOTPyramid pyramid(models()[0].boxSize_, models_[0].parts().size(), interval, scenes[i].resolution());


        pyramid.createFilteredPyramid(cloud, models_[0].parts()[0].filter,
                min, max, 0, 20);
//        pyramid.createFullPyramid(cloud, min, max, 50);

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

//        #pragma omp parallel for
        for (int lvl = 0; lvl < pyramid.levels().size(); ++lvl) {
            const double scale = 1 / pow(2.0, static_cast<double>(lvl) / interval);

            int rows = 0;
            int cols = 0;
            int depths = 0;

            if (!zero_ && scores[lvl].size()) {
                depths = scores[lvl][0].depths();
                rows = scores[lvl][0].rows();
                cols = scores[lvl][0].cols();
            }
            else if (lvl >= interval && pyramid.levels()[lvl].size()) {
                depths = static_cast<int>(pyramid.levels()[lvl][0].depths()) - maxSize()(0)*scale + 1;
                rows = static_cast<int>(pyramid.levels()[lvl][0].rows()) - maxSize()(1)*scale + 1;
                cols = static_cast<int>(pyramid.levels()[lvl][0].cols()) - maxSize()(2)*scale + 1;
            }

            cout<<"Mix::negLatentSearch sorting ..."<<endl;

            vector<ScoreStruct> bestNeg;
            for (int box = 0; box < pyramid.levels()[lvl].size(); ++box) {
                Rectangle& bndbox = pyramid.rectangles_[lvl][box];
                for (int z = 0; z < depths; ++z) {
                    for (int y = 0; y < rows; ++y) {
                        for (int x = 0; x < cols; ++x) {
                            bool intersection = false;
                            for (int k = 0; k < intersectors.size(); ++k) {
                                if (intersectors[k](bndbox.cloud(), bndbox.volume())){
                                    intersection = true;
                                }
                            }
                            if(zero_){
                                if(!intersection){
                                    bestNeg.push_back( ScoreStruct( 0, lvl, box, z, y, x));
                                }
                            } else{
                                if(!intersection && scores[lvl][box]()(z, y, x) > -1/*negatives.last().first.parts()[0].deformation(7)*/){
                                    bestNeg.push_back( ScoreStruct( scores[lvl][box]()(z, y, x), lvl, box, z, y, x));
                                }
                            }
                        }
                    }
                }
            }

            sort( bestNeg.begin(), bestNeg.end());
            if(bestNeg.size()>2) cout<<"neg sorted : "<<bestNeg[0].score<<" / "<<bestNeg[bestNeg.size()-1].score<<endl;

            int prevSize = negatives.size();
            negatives.resize(prevSize+bestNeg.size());

            #pragma omp parallel for
            for (int n = 0; n < bestNeg.size(); ++n) {
                Model sample;
                const int argmax = 0;

                ScoreStruct& neg = bestNeg[n];

                models_[argmax].initializeSample(pyramid, neg.box, neg.z, neg.y, neg.x,
                                                 neg.lvl, sample, zero_ ? 0 : &positions[argmax]);

                if (!sample.empty()) {

                    // Store all the information about the sample in the offset and
                    // deformation of its root
                    sample.parts()[0].offset(0) = i;
                    sample.parts()[0].offset(1) = neg.lvl;
                    sample.parts()[0].offset(2) = neg.box;
                    sample.parts()[0].offset(3) = 0;
                    sample.parts()[0].deformation(0) = 0;
                    sample.parts()[0].deformation(1) = 0;
                    sample.parts()[0].deformation(2) = 0;
                    sample.parts()[0].deformation(3) = 0;
                    sample.parts()[0].deformation(4) = 0;
                    sample.parts()[0].deformation(5) = 0;
                    sample.parts()[0].deformation(6) = 0;
                    sample.parts()[0].deformation(7) = zero_ ? 0.0 : neg.score;

//                    cout<<"neg sample with score : "
//                       <<sample.parts()[0].deformation(7)<<endl;
//                    bool toPush = true;
//                    vector<pair<Model, int> >::iterator it;
//                    for( it = negatives.begin(); it != negatives.end();){
//                        if( it->first.parts()[0].deformation(7) > sample.parts()[0].deformation(7)){
//                            ++it;
//                        } else if( (it->first == sample)){///doesnt work
////                            cout<<"try add same neg sample"<<endl;
////                            cout<<"it->first.parts()[0].offset : "
////                               << it->first.parts()[0].offset.transpose()<<endl;
////                            cout<<"sample.parts()[0].offset : "
////                               << sample.parts()[0].offset.transpose()<<endl;
//                            toPush = false;
//                            break;
//                        } else if( it+1 != negatives.end()){
////                            cout<<"insert neg sample"<<endl;
//                            negatives.insert(it, make_pair(sample, argmax));
//                            toPush = false;
//                            break;
//                        }
//                    }

//                    if( negatives.size() < maxNegatives && toPush){///doesnt work
//                        cout<<"push_back neg sample"<<endl;
                        negatives[prevSize+n] = make_pair(sample, argmax);
//                    }



                }
            }
        }
        if (negatives.size() > maxNegatives){
            sort( negatives.begin(), negatives.end(), NegSort());
            negatives.resize(maxNegatives);
            if(negatives.back().first.parts()[0].deformation(7) > -0.5) return;
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

Matrix3f Mixture::getRotation(Vector4f orientationFrom, Vector4f orientationTo){

    Matrix3f rotation ;/*= orientationTo * orientationFrom.inverse();*/

    cout<<"orientationFrom : "<<orientationFrom.transpose()<<endl;
    cout<<"orientationTo : "<<orientationTo.transpose()<<endl;

    cout<<"transform : "<<endl<<rotation<<endl;

    return rotation;
}

Vector3i Mixture::FilterSizes(int nbComponents, const vector<Scene> & scenes,
                                             Object::Name name, int interval)
{
	// Early return in case the filters or the dataset are empty
    if ((nbComponents <= 0) || scenes.empty()){
        cerr<<"nbComponents <= 0 or scenes.empty()"<<endl;
        return Vector3i();
    }

    const float scale = 1 / pow(2.0, interval);
	// Sort the aspect ratio of all the (non difficult) samples
    vector<Rectangle> rects;
//    Eigen::Vector4f baseLocalPose;

//    for (int i = 0; i < scenes.size(); ++i) {
//        for (int j = 0; j < scenes[i].objects().size(); ++j) {
//            const Object & obj = scenes[i].objects()[j];

//            if ((obj.name() == name) && !obj.difficult()){
//                baseLocalPose = scenes[i].localPose()[j];
//                cout<<"Mixture::baseLocalPose : "<<baseLocalPose<<endl;
//                break;
//            }
//        }
//        if( baseLocalPose != Eigen::Vector4f()){
//            break;
//        }
//    }

	for (int i = 0; i < scenes.size(); ++i) {
		for (int j = 0; j < scenes[i].objects().size(); ++j) {
			const Object & obj = scenes[i].objects()[j];
			
            if ((obj.name() == name) && !obj.difficult()){
//                Vector3f sizes = getNormalizeTransform( scenes[i].localPose()[j], baseLocalPose) * obj.bndbox().size();
//                Rectangle bndbox( obj.bndbox().origin(), sizes);
//                rects.push_back( bndbox);
//                cout<<"Mixture::FilterSizes rects before norm : "<<obj.bndbox()<<endl;
//                cout<<"Mixture::FilterSizes rects after norm : "<<bndbox<<endl;

                rects.push_back( obj.bndbox());
                cout<<"Mixture::FilterSizes rects : "<<obj.bndbox()<<endl;

            }
		}
	}
	
    // Early return if there is no object
    if (rects.empty()){
        cerr<<"rects.empty()"<<endl;
        return Vector3i();
    }

    // Sort the aspect ratio of all the samples
//    sort(rects.begin(), rects.end());
//    reverse(rects.begin(), rects.end());

    // For each mixture model
    vector<int> references(nbComponents+1);
    Vector3i sizes(0,0,0);

    for (int i = 0; i <= nbComponents; ++i){
        references[i] = rects.size() * i / nbComponents;
    }

    for (int i = 0; i < references.size(); ++i){
        cout<<"Mixture::FilterSizes ref : "<<references[i]<<endl;
    }
	
//    for (int i = 0; i < nbComponents; ++i) {
        Vector3f boxSize(0,0,0);

        for (int j = references[0]; j < references[1]; ++j) {
            boxSize += rects[j].size();
        }


        sizes(0) = ceil( boxSize(0) / (references[1]-references[0]) * scale / scenes[0].resolution());
        sizes(1) = ceil( boxSize(1) / (references[1]-references[0]) * scale / scenes[0].resolution());
        sizes(2) = ceil( boxSize(2) / (references[1]-references[0]) * scale / scenes[0].resolution());
//    }
        cout<<"Mixture::FilterSizes boxSize : "<<endl<<sizes<<endl;

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
