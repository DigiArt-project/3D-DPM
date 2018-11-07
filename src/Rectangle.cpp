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

#include "Rectangle.h"


using namespace FFLD;
using namespace std;

Rectangle::Rectangle() : origin_( 0, 0, 0), boxSizes_(0, 0, 0), cloud_( 8,1,PointType()),
    tform_(Eigen::Matrix4f::Identity()), volume_(0)
{
//    PointType p = PointType();
//    p.z = 0;
//    p.y = 0;
//    p.x = 0;
//    cloud_.at(0) = p;
//    cloud_.at(1) = p;
//    cloud_.at(2) = p;
//    cloud_.at(3) = p;
//    cloud_.at(4) = p;
//    cloud_.at(5) = p;
//    cloud_.at(6) = p;
//    cloud_.at(7) = p;
}

Rectangle::Rectangle(const Rectangle& rect)
    : origin_( rect.origin_), boxSizes_(rect.boxSizes_), cloud_( rect.cloud_.size(),1,PointType()),// rect.cloud_),
      tform_( rect.tform_), volume_(rect.volume_)
{
    for(int i = 0; i < rect.cloud_.size(); ++i){
        cloud_.points[i] = rect.cloud_.points[i];
    }
}

Rectangle::Rectangle(Eigen::Vector3f origin, Eigen::Vector3f boxSizes, Eigen::Matrix4f tform) :
    origin_( origin), boxSizes_(boxSizes), cloud_( 8,1,PointType()),
    tform_(tform)
{
    volume_ = boxSizes_(0) * boxSizes_(1) * boxSizes_(2);

    PointCloudT cloud (8,1,PointType());
    PointType p = PointType();
    p.z = origin(0);
    p.y = origin(1);
    p.x = origin(2);
    cloud.at(0) = p;
    p.z = origin(0)+boxSizes(0);
    p.y = origin(1);
    p.x = origin(2);
    cloud.at(1) = p;
    p.z = origin(0);
    p.y = origin(1)+boxSizes(1);
    p.x = origin(2);
    cloud.at(2) = p;
    p.z = origin(0)+boxSizes(0);
    p.y = origin(1)+boxSizes(1);
    p.x = origin(2);
    cloud.at(3) = p;
    p.z = origin(0);
    p.y = origin(1);
    p.x = origin(2)+boxSizes(2);
    cloud.at(4) = p;
    p.z = origin(0)+boxSizes(0);
    p.y = origin(1);
    p.x = origin(2)+boxSizes(2);
    cloud.at(5) = p;
    p.z = origin(0);
    p.y = origin(1)+boxSizes(1);
    p.x = origin(2)+boxSizes(2);
    cloud.at(6) = p;
    p.z = origin(0)+boxSizes(0);
    p.y = origin(1)+boxSizes(1);
    p.x = origin(2)+boxSizes(2);
    cloud.at(7) = p;

    pcl::transformPointCloud (cloud, cloud_, tform);

}

//Rectangle::~Rectangle()
//{
////    cloud_ = NULL;
////    delete [] cloud_;

//}

Eigen::Vector3f Rectangle::origin() const
{
    return origin_;
}

Eigen::Vector3f Rectangle::size() const
{
    return boxSizes_;
}

float Rectangle::origin( int i) const
{
    return origin_(i);
}

float Rectangle::size( int i) const
{
    return boxSizes_(i);
}

PointCloudT Rectangle::cloud() const{
    return cloud_;
}

PointType Rectangle::cloud( int index) const{
    return cloud_.points[index];
}

//void Rectangle::setCloud( PointCloudPtr cloud){
//    cloud_ = cloud;
//}

Eigen::Matrix4f Rectangle::transform() const{
    return tform_;
}

bool Rectangle::empty() const
{
    return (boxSizes_(0) <= 0) || (boxSizes_(1) <= 0) || (boxSizes_(2) <= 0 ) || (volume_ <=0);
}

float Rectangle::volume() const
{
    return volume_;
}

bool Rectangle::operator<(const Rectangle & rect) const{
    return volume() < rect.volume() && !( rect.volume() < volume());
}

ostream & FFLD::operator<<(ostream & os, const Rectangle & rect)
{
    os << rect.origin(0) << ' ' << rect.origin()(1) << ' ' << rect.origin()(2) << ' '
              << rect.size(0) << ' ' << rect.size(1) << ' ' << rect.size(2) << ' ';
    for(int i=0; i< rect.transform().rows(); ++i){
        for(int j=0; j< rect.transform().cols(); ++j){
            os << rect.transform()(i,j) << ' ';
        }
    }
    return os;
}

istream & FFLD::operator>>(istream & is, Rectangle & rect)
{
    Eigen::Vector3f origin, boxSizes;
    Eigen::Matrix4f tform;
	
    is >> origin(0) >> origin(1) >> origin(2)
       >> boxSizes(0) >> boxSizes(1) >> boxSizes(2);
    for(int i=0; i< tform.rows(); ++i){
        for(int j=0; j< tform.cols(); ++j){
            is >> tform(i,j);
        }
    }
	
    rect = Rectangle(origin, boxSizes, tform);
	
	return is;
}
