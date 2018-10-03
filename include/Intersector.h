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

#ifndef FFLD_INTERSECTOR_H
#define FFLD_INTERSECTOR_H

#include "Rectangle.h"

#include <algorithm>

namespace FFLD
{
/// Functor used to test for the intersection of two rectangles according to the Pascal criterion
/// (area of intersection over area of union).
class Intersector
{
public:
	/// Constructor.
	/// @param[in] reference The reference rectangle.
	/// @param[in] threshold The threshold of the criterion.
	/// @param[in] felzenszwalb Use Felzenszwalb's criterion instead of the Pascal one (area of
	/// intersection over area of second rectangle). Useful to remove small detections inside bigger
	/// ones.
	Intersector(const Rectangle & reference, double threshold = 0.5, bool felzenszwalb = false) :
    reference_(reference), threshold_(threshold), felzenszwalb_(felzenszwalb),
      intersectionCloud_(new PointCloudT())
    {
	}
	
	/// Tests for the intersection between a given rectangle and the reference.
	/// @param[in] rect The rectangle to intersect with the reference.
	/// @param[out] score The score of the intersection.
	bool operator()(const Rectangle & rect, double * score = 0) const
	{
        if (score)
            *score = 0.0;

        cout<<"Intersector ..."<<endl;

        vector<int> ptsIndicesRec;
        vector<int> ptsIndicesRef;
        getIntersectionPts( rect.cloud(), ptsIndicesRec, ptsIndicesRef);
        cout<<"Intersector:: getIntersectionPts done"<<endl;
        cout<<"Intersector:: ptsIndicesRec.size() : "<<ptsIndicesRec.size()<<endl;
        cout<<"Intersector:: ptsIndicesRef.size() : "<<ptsIndicesRef.size()<<endl;


        vector<PointType> ptsIntersectionRec = computeIntersectionPtsAt( rect.cloud(), ptsIndicesRec,
                                                                         reference_.cloud());
        cout<<"Intersector:: ptsIntersectionRec.size() : "<<ptsIntersectionRec.size()<<endl;

        vector<PointType> ptsIntersectionRef = computeIntersectionPtsAt( reference_.cloud(), ptsIndicesRef,
                                                                         rect.cloud());
        cout<<"Intersector:: ptsIntersectionRef.size() : "<<ptsIntersectionRef.size()<<endl;

        //Remove duplicates
        vector<PointType> ptsIntersection = ptsIntersectionRec;

        for( int i = 0; i < ptsIntersectionRef.size(); ++i){
            ptsIntersection.push_back( ptsIntersectionRef[i]);
        }
        sort( ptsIntersection.begin(), ptsIntersection.end(), pointTypeIsInferior);

        vector<PointType>::iterator it;
        for( it = ptsIntersection.begin(); it != ptsIntersection.end()-1; /*++it*/){
            if( pointTypeIsEqual( *it, *(it+1))){
                it = ptsIntersection.erase( it);
            } else{
                ++it;
            }
        }

        PointCloudPtr intersectionCloud(
                            new PointCloudT( ptsIntersection.size(), 1, PointType()));

        for( int i = 0; i < ptsIntersection.size(); ++i){
            intersectionCloud->points[i] = ptsIntersection[i];
        }
        cout<<"Intersector:: intersectionCloud->size() : "<<intersectionCloud->size()<<endl;

        float intersectionVolume = 0;
        const float cubeVolume = rect.volume();
        if( intersectionCloud->size() > 3){
            pcl::ConvexHull<PointType> cHull;
            pcl::PointCloud<PointType> cHullCloud;
            cHull.setInputCloud(intersectionCloud);
            cHull.setComputeAreaVolume(true);
            cHull.reconstruct (cHullCloud);
            cout<<"Intersector:: reconstruct cHull done"<<endl;

            intersectionVolume = cHull.getTotalVolume();
            cout<<"Intersector:: intersectionVolume : "<<intersectionVolume<<endl;
            cout<<"Intersector:: cubeVolume : "<<cubeVolume<<endl;

            for(int i=0; i<intersectionCloud->size();++i){
                cout<<"Pt["<<i<<"] : "<<intersectionCloud->points[i].x<<" / "
                   <<intersectionCloud->points[i].y<<" / "
                  <<intersectionCloud->points[i].z<<endl;
            }
        }

        *intersectionCloud_ = *intersectionCloud;

        if (felzenszwalb_) {
            if (intersectionVolume > cubeVolume * threshold_ && cubeVolume) {
                if (score)
                    *score = intersectionVolume / cubeVolume;
                cout<<"Intersector::done"<<endl;

                return true;
            }
        }
        else {
            const float referenceVolume = reference_.volume();
            const float unionVolume = referenceVolume + cubeVolume - intersectionVolume;
            std::cout<<"intersectionVolume : "<<intersectionVolume<<std::endl;
            std::cout<<"unionVolume * threshold_ : "<<unionVolume * threshold_<<std::endl;
            std::cout<<"unionVolume : "<<unionVolume<<std::endl;

            if (intersectionVolume > unionVolume * threshold_ && unionVolume) {
                if (score)
                    *score = intersectionVolume / unionVolume;
                cout<<"Intersector::done"<<endl;

                return true;
            }
        }
        cout<<"Intersector::done"<<endl;

		return false;
	}
	
    PointCloudPtr intersectionCloud_;

private:

    void getIntersectionPts( PointCloudPtr rectCloud, vector<int>& ptsIndicesRec,
                                      vector<int>& ptsIndicesRef) const{
        PointType minRec;
        PointType maxRec;
        pcl::getMinMax3D(*rectCloud, minRec, maxRec);
        PointType minRef;
        PointType maxRef;
        pcl::getMinMax3D(*reference_.cloud(), minRef, maxRef);

        Eigen::Vector4f startRec( minRec.x, minRec.y, minRec.z, 1);
        Eigen::Vector4f endRec( maxRec.x, maxRec.y, maxRec.z, 1);
        Eigen::Vector4f startRef( minRef.x, minRef.y, minRef.z, 1);
        Eigen::Vector4f endRef( maxRef.x, maxRef.y, maxRef.z, 1);

        pcl::getPointsInBox(*rectCloud, startRef, endRef, ptsIndicesRec);
        pcl::getPointsInBox(*reference_.cloud(), startRec, endRec, ptsIndicesRef);
    }

    vector<int> getAdjacentPts(int index) const{
        vector<int> adjacentPts(3);
        if( index%2 == 0){
            adjacentPts[0] = index+1;
            if( index%4 == 0){
                adjacentPts[1] = index+2;
            } else{
                adjacentPts[1] = index-2;
            }
            adjacentPts[2] = (index+4)%8;

        } else{
            adjacentPts[0] = index-1;
            if( index%4 == 1){
                adjacentPts[1] = index+2;
            } else{
                adjacentPts[1] = index-2;
            }
            adjacentPts[2] = (index+4)%8;
        }


//        cout<<"AdjacentPts of : "<<index<<" are : ";
//        for(int i=0;i<adjacentPts.size();++i){
//            cout<<adjacentPts[i]<<" / ";
//        }
//        cout<<endl;
        return adjacentPts;
    }

    bool findIntersection( Eigen::Vector3f origin1, Eigen::Vector3f direction1,
                           Eigen::Vector3f planePt2, Eigen::Vector3f planeNormal2, PointType* p) const{

        float den = planeNormal2.dot( direction1);
        if( abs(den) <= 0.001){
            return false;
        }

        float t = planeNormal2.dot( planePt2 - origin1) / ( planeNormal2.dot( direction1));

        if( t <= 0){
            return false;
        }

//        cout<<"findIntersection den : "<<den<<endl;
//        cout<<"findIntersection t : "<<t<<endl;
//        cout<<"findIntersection origin1 : "<<origin1<<endl;
//        cout<<"findIntersection direction1 : "<<direction1<<endl;

        p->x = origin1(0) + direction1(0) * t;
        p->y = origin1(1) + direction1(1) * t;
        p->z = origin1(2) + direction1(2) * t;
        return true;
    }

    vector<PointType> computeIntersectionPtsAt( PointCloudPtr cloud1, vector<int> indices1,
                                                PointCloudPtr cloud2) const{
        vector<PointType> res;
        PointType* p = new PointType();

        for( int t = 0; t < indices1.size(); ++t){
            int index1 = indices1[t];
            bool originIsInclude = false;
            Eigen::Vector3f origin1(  cloud1->points[index1].x,
                                      cloud1->points[index1].y,
                                      cloud1->points[index1].z);
            vector<int> adjacentPts1 = getAdjacentPts(index1);

            for( int i = 0; i < adjacentPts1.size(); ++i){

                Eigen::Vector3f direction1(
                            cloud1->points[adjacentPts1[i]].x - cloud1->points[index1].x,
                            cloud1->points[adjacentPts1[i]].y - cloud1->points[index1].y,
                            cloud1->points[adjacentPts1[i]].z - cloud1->points[index1].z);

                //for each face of the cube of cloud2, findIntersection with cube of cloud1
                for( int j = 0; j < 2; ++j){
                    int index2 = j * 7;//index 0 or 7
                    Eigen::Vector3f planePt2( cloud2->points[index2].x,
                                              cloud2->points[index2].y,
                                              cloud2->points[index2].z);
                    vector<int> adjacentPts2 = getAdjacentPts(index2);

                    for( int k = 0; k < adjacentPts2.size(); ++k){//size = 3

//                        Eigen::Vector3f u(
//                                    cloud2->points[adjacentPts2[k]].x - cloud2->points[index2].x,
//                                    cloud2->points[adjacentPts2[k]].y - cloud2->points[index2].y,
//                                    cloud2->points[adjacentPts2[k]].z - cloud2->points[index2].z);
//                        Eigen::Vector3f v(
//                                    cloud2->points[adjacentPts2[(k+1)%adjacentPts2.size()]].x - cloud2->points[index2].x,
//                                    cloud2->points[adjacentPts2[(k+1)%adjacentPts2.size()]].y - cloud2->points[index2].y,
//                                    cloud2->points[adjacentPts2[(k+1)%adjacentPts2.size()]].z - cloud2->points[index2].z);
//                        Eigen::Vector3f planeNormal2 = u.cross(v);
                        Eigen::Vector3f planeNormal2(
                                    cloud2->points[index2].x - cloud2->points[adjacentPts2[k]].x,
                                    cloud2->points[index2].y - cloud2->points[adjacentPts2[k]].y,
                                    cloud2->points[index2].z - cloud2->points[adjacentPts2[k]].z);


                        if( findIntersection( origin1, direction1, planePt2, planeNormal2, p)){
                            res.push_back(*p);
                            cout<<"add intersection pt : "<<*p<<endl;

//                            cout<<"origin pt : "<<cloud1->points[index1]<<endl;
//                            cout<<"planeNormal2 : "<<planeNormal2.transpose()<<endl;
//                            cout<<"direction1 : "<<direction1.transpose()<<endl;

//                            cout<<"originIsInclude score : "<<planeNormal2.dot( direction1)<<endl;
                            if( planeNormal2.dot( direction1) >= 0){
                                originIsInclude = true;
                            }
                        }
                    }
                }
            }
            if(originIsInclude){
                cout<<"add origin pt : "<<cloud1->points[index1]<<endl;
                res.push_back(cloud1->points[index1]);
            }
        }
        return res;

    }

	Rectangle reference_;
	double threshold_;
	bool felzenszwalb_;
};
}

#endif
