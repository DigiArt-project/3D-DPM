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

//        cout<<"Intersector ..."<<endl;
//        PointCloudPtr intersectionCloud(new PointCloudT());
//        intersectionCloud->width = 3;
//        intersectionCloud->height = 1;
//        intersectionCloud->points.resize( 3);
        vector<int> ptsIndices = {0,1,2,4,7,5,3,6};//{0,5,3,6};
//        getIntersectionPts( rect.cloud(), ptsIndicesRec, ptsIndicesRef);
//        cout<<"Intersector:: getIntersectionPts done"<<endl;
//        cout<<"Intersector:: ptsIndicesRec.size() : "<<ptsIndicesRec.size()<<endl;
//        cout<<"Intersector:: ptsIndicesRef.size() : "<<ptsIndicesRef.size()<<endl;


        vector<PointType> ptsIntersectionRec = computeIntersectionPtsAt( rect.cloud(), ptsIndices,
                                                                         reference_.cloud());
//        cout<<"Intersector:: ptsIntersectionRec.size() : "<<ptsIntersectionRec.size()<<endl;

        vector<PointType> ptsIntersectionRef = computeIntersectionPtsAt( reference_.cloud(), ptsIndices,
                                                                         rect.cloud());
//        cout<<"Intersector:: ptsIntersectionRef.size() : "<<ptsIntersectionRef.size()<<endl;

        //Remove duplicates
        vector<PointType> ptsIntersection = ptsIntersectionRec;

        for( int i = 0; i < ptsIntersectionRef.size(); ++i){
            ptsIntersection.push_back( ptsIntersectionRef[i]);
        }

        if( ptsIntersection.size() < 3){
            return false;
        }

//        cout<<"Intersector:: ptsIntersection.size() before remove duplicates : "<<ptsIntersection.size()<<endl;

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
//        cout<<"Intersector:: ptsIntersection.size() after remove duplicates : "<<ptsIntersection.size()<<endl;


        for( int i = 0; i < ptsIntersection.size(); ++i){
//            cout<<"Intersector:: i : "<<i<<endl;
            intersectionCloud->points[i] = ptsIntersection[i];
        }

        //Not thread safe, check thread access !!!!!!!!!!
        float intersectionVolume = 0;
        const float cubeVolume = rect.volume();
        if( intersectionCloud->size() > 3){
            pcl::ConvexHull<PointType> cHull;
            pcl::PointCloud<PointType> cHullCloud;
            cHull.setInputCloud(intersectionCloud);
            cHull.setComputeAreaVolume(true);
            cHull.setDimension(3);
            cHull.reconstruct (cHullCloud);
//            cout<<"Intersector:: reconstruct cHull done"<<endl;

            intersectionVolume = cHull.getTotalVolume();
            cout<<"Intersector:: intersectionCloud->size() : "<<intersectionCloud->size()<<endl;
            cout<<"Intersector:: intersectionVolume : "<<intersectionVolume<<endl;
//            cout<<"Intersector:: cubeVolume : "<<cubeVolume<<endl;

//            for(int i=0; i<intersectionCloud->size();++i){
//                cout<<"Pt["<<i<<"] : "<<intersectionCloud->points[i].x<<" / "
//                   <<intersectionCloud->points[i].y<<" / "
//                  <<intersectionCloud->points[i].z<<endl;
//            }
        }

        *intersectionCloud_ = *intersectionCloud;

        if (felzenszwalb_) {
            if (intersectionVolume > cubeVolume * threshold_ && cubeVolume) {
                if (score)
                    *score = intersectionVolume / cubeVolume;
//                cout<<"Intersector::done"<<endl;

                return true;
            }
        }
        else {
            const float referenceVolume = reference_.volume();
            const float unionVolume = referenceVolume + cubeVolume - intersectionVolume;
//            std::cout<<"intersectionVolume : "<<intersectionVolume<<std::endl;
//            std::cout<<"unionVolume * threshold_ : "<<unionVolume * threshold_<<std::endl;
//            std::cout<<"unionVolume : "<<unionVolume<<std::endl;

            if (intersectionVolume > unionVolume * threshold_ && unionVolume) {
                if (score)
                    *score = intersectionVolume / unionVolume;
//                cout<<"Intersector::done"<<endl;

                return true;
            }
        }
//        cout<<"Intersector::done"<<endl;

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

    vector<int> getFacePts( int index1, int index2, int index3) const{
        vector<int> res(4);
        res[0] = index1;//= 0 or 7
        res[1] = index2;
        res[2] = index3;
//        if( index1 == 0){
//            res[3] = index2+index3;
////            if( ( index2 == 1 && index3 == 2) || ( index2 == 2 && index3 == 1)){
////                res[3] = 3;
////            }
////            if( ( index2 == 1 && index3 == 4) || ( index2 == 4 && index3 == 1)){
////                res[3] = 5;
////            }
////            if( ( index2 == 4 && index3 == 2) || ( index2 == 2 && index3 == 4)){
////                res[3] = 6;
////            }
//        } else{//index1 == 7
//            res[3] = index2+index3-index1;
//        }
        res[3] = index2+index3-index1;
        return res;
    }

    bool belongToFace( PointType p, PointCloudPtr cloud, int index1, int index2, int index3) const{
        vector<int> facePts = getFacePts( index1, index2, index3);
        Eigen::Vector3f center(0,0,0);
        Eigen::Vector3f val(0,0,0);

        cout<<"facePts : ";
        for( int i = 0; i < facePts.size(); ++i){
            cout<<facePts[i]<<" / ";
            center(0) += cloud->points[facePts[i]].x;
            center(1) += cloud->points[facePts[i]].y;
            center(2) += cloud->points[facePts[i]].z;
            val(0) += cloud->points[facePts[i]].x;
            val(1) += cloud->points[facePts[i]].y;
            val(2) += cloud->points[facePts[i]].z;
        }
        cout<<endl;
        cout<<"facePts : ";
        for( int i = 0; i < facePts.size(); ++i){
            cout<<cloud->points[facePts[i]]<<" / "<<endl;
        }
        cout<<endl;
        center /= facePts.size();
        float sum1 = abs(p.x - center(0)) + abs(p.y - center(1)) + abs(p.z - center(2));
        float sum2 = abs(center(0)) + abs(center(1)) + abs(center(2));

        cout<<"center : "<<endl<<center<<endl;
        cout<<"pt to add : "<<p<<endl;
        cout<<"sum1 : "<<sum1<<endl;
        cout<<"sum2 : "<<sum2<<endl;

        if( sum1 <= sum2){
            return true;
        }
        return false;
    }

    bool belongToCube( PointType p, PointCloudPtr cloud1, vector<Eigen::Vector3f> directions1,
                       int index1, vector<int> adjacentPts1, float epsilon = 0.001) const{

        Eigen::Vector3f pt( p.x, p.y, p.z);
        Eigen::Vector3f pIndex1( cloud1->points[index1].x, cloud1->points[index1].y, cloud1->points[index1].z);
//        cout<<"pt = "<<pt.transpose()<<endl;
//        cout<<"pIndex1 = "<<pIndex1<<endl;


        for( int i = 0; i < directions1.size(); ++i){
            Eigen::Vector3f pAdj1(  cloud1->points[adjacentPts1[i]].x,
                                    cloud1->points[adjacentPts1[i]].y,
                                    cloud1->points[adjacentPts1[i]].z);
            float res = directions1[i].dot( pt);

//            cout<<"pAdj1 = "<<pAdj1.transpose()<<endl;
//            cout<<"directions1[i] = "<<directions1[i].transpose()<<endl;
//            cout<<"res = "<<res<<endl;
//            cout<<"limLow = "<<directions1[i].dot( pIndex1)<<endl;
//            cout<<"limHigh = "<<directions1[i].dot( pAdj1)<<endl;

            if( res + epsilon < directions1[i].dot( pIndex1) || res > directions1[i].dot( pAdj1) + epsilon){
//                cout<<"belongToCube = false"<<endl;
                return false;
            }
        }
//        cout<<"belongToCube = true"<<endl;
        return true;
    }

    bool findIntersection( Eigen::Vector3f origin1, Eigen::Vector3f direction1,
                           Eigen::Vector3f planePt2, Eigen::Vector3f planeNormal2, PointType* p) const{

        float den = planeNormal2.dot( direction1);
        if( abs(den) <= 0.001){
            return false;
        }

        float t = planeNormal2.dot( planePt2 - origin1) / ( planeNormal2.dot( direction1));

        if( t < 0 || t > 1){
            return false;
        }

//        cout<<"findIntersection num : "<<planeNormal2.dot( planePt2 - origin1)<<endl;
//        cout<<"findIntersection den : "<<den<<endl;
//        cout<<"findIntersection planePt2 - origin1 : "<<planePt2 - origin1<<endl;
//        cout<<"findIntersection t : "<<t<<endl;
//        cout<<"findIntersection origin1 : "<<origin1<<endl;
//        cout<<"findIntersection direction1 : "<<direction1<<endl;
//        cout<<"findIntersection planePt2 : "<<planePt2<<endl;
//        cout<<"findIntersection planeNormal2 : "<<planeNormal2<<endl;

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

            vector<Eigen::Vector3f> directions1(adjacentPts1.size());//size = 3
            for( int i = 0; i < adjacentPts1.size(); ++i){
                directions1[i] = Eigen::Vector3f( cloud1->points[adjacentPts1[i]].x - cloud1->points[index1].x,
                                cloud1->points[adjacentPts1[i]].y - cloud1->points[index1].y,
                                cloud1->points[adjacentPts1[i]].z - cloud1->points[index1].z);
            }

            for( int i = 0; i < adjacentPts1.size(); ++i){

                Eigen::Vector3f direction1 = directions1[i];

                //for each face of the cube of cloud2, findIntersection with cube of cloud1
                for( int j = 0; j < 2; ++j){
                    int index2 = j * 7;//index 0 or 7
                    Eigen::Vector3f planePt2( cloud2->points[index2].x,
                                              cloud2->points[index2].y,
                                              cloud2->points[index2].z);
                    vector<int> adjacentPts2 = getAdjacentPts(index2);

                    vector<Eigen::Vector3f> directions2(adjacentPts2.size());//size = 3
                    for( int i = 0; i < adjacentPts2.size(); ++i){
                        directions2[i] = Eigen::Vector3f( cloud2->points[adjacentPts2[i]].x - cloud2->points[index2].x,
                                        cloud2->points[adjacentPts2[i]].y - cloud2->points[index2].y,
                                        cloud2->points[adjacentPts2[i]].z - cloud2->points[index2].z);
                    }

                    for( int k = 0; k < adjacentPts2.size(); ++k){//size = 3

                        Eigen::Vector3f planeNormal2(
                                    cloud2->points[index2].x - cloud2->points[adjacentPts2[k]].x,
                                    cloud2->points[index2].y - cloud2->points[adjacentPts2[k]].y,
                                    cloud2->points[index2].z - cloud2->points[adjacentPts2[k]].z);


                        if( findIntersection( origin1, direction1, planePt2, planeNormal2, p)){
//                            if( belongToFace(*p, cloud2, index2, adjacentPts2[(k+1)%adjacentPts2.size()],
//                                             adjacentPts2[(k+2)%adjacentPts2.size()])){
                            if( belongToCube(*p, cloud2, directions2, index2, adjacentPts2)){
                                res.push_back(*p);
//                                cout<<"add intersection pt : "<<*p<<endl;

//                                cout<<"origin pt1 : "<<cloud1->points[index1]<<endl;
//                                cout<<"origin pt2 : "<<cloud2->points[index2]<<endl;
//                                cout<<"planeNormal2 : "<<planeNormal2.transpose()<<endl;
//                                cout<<"direction1 : "<<direction1.transpose()<<endl;

//                                cout<<"originIsInclude score : "<<planeNormal2.dot( direction1)<<endl;
                                if( belongToCube(cloud1->points[index1], cloud2, directions2, index2, adjacentPts2) /*&& planeNormal2.dot( direction1) <= 2*/){
                                    originIsInclude = true;
                                }
                            }
                        }
                    }
                }
            }
            if(originIsInclude){
//                cout<<"add origin pt : "<<cloud1->points[index1]<<endl;
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
