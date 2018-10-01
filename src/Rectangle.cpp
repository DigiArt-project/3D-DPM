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

Rectangle::Rectangle() : origin_( 0, 0, 0), diagonal_( 0, 0, 0), width_(0), height_(0), depth_(0),
    resolution_(0.0)
{
}
Rectangle::Rectangle(const Rectangle& rect)
    : origin_( rect.origin_), width_(rect.width_), height_(rect.height_), depth_(rect.depth_),
      diagonal_( rect.diagonal_), volume_(rect.volume_), resolution_(rect.resolution_)
{}

Rectangle::Rectangle(float depth, float height, float width, float resolution) :
    origin_( 0, 0, 0), width_(width), height_(height), depth_(depth),
    diagonal_( depth_, height_, width_), resolution_(resolution)
{
    volume_ = volume();
}

Rectangle::Rectangle(Eigen::Vector3f origin, float depth, float height, float width, float resolution) :
    origin_( origin), width_(width), height_(height), depth_(depth), resolution_(resolution)
{
    diagonal_ = Eigen::Vector3f( origin_(0) + depth_, origin_(1) + height_, origin_(2) + width_);
    volume_ = volume();
}

Eigen::Vector3f Rectangle::origin() const
{
    return origin_;
}

Eigen::Vector3f Rectangle::diagonal() const
{
    return diagonal_;
}

void Rectangle::setOrigin(Eigen::Vector3f origin)
{
    origin_ = origin;
}

void Rectangle::setDiagonal(Eigen::Vector3f diagonal)
{
    diagonal_ = diagonal;
}

float Rectangle::width() const
{
	return width_;
}

void Rectangle::setWidth(float width)
{
	width_ = width;
}

float Rectangle::height() const
{
	return height_;
}

void Rectangle::setHeight(float height)
{
	height_ = height;
}

float Rectangle::depth() const
{
    return depth_;
}

void Rectangle::setDepth(float depth)
{
    depth_ = depth;
}

float Rectangle::right() const
{
    return diagonal_(2);
}

float Rectangle::left() const
{
    return origin_(2);
}

float Rectangle::top() const
{
    return origin_(1);
}

float Rectangle::bottom() const
{
    return diagonal_(1);
}

float Rectangle::front() const
{
    return origin_(0);
}

float Rectangle::back() const
{
    return diagonal_(0);
}

float Rectangle::resolution() const
{
    return resolution_;
}

void Rectangle::setResolution( float resolution)
{
    resolution_ = resolution;
}

void Rectangle::setLeft(float left)
{
    origin_(2) = left;
    setWidth( right() - left);
}

void Rectangle::setRight(float right)
{
    diagonal_(2) = right;
    setWidth( right - left());
}

void Rectangle::setBottom(float bottom)
{
    diagonal_(1) = bottom;
    setHeight( top() - bottom);
}

void Rectangle::setTop(float top)
{
    origin_(1) = top;
    setHeight( top - bottom());
}

void Rectangle::setBack(float back)
{
    diagonal_(0) = back;
    setDepth( front() - back);
}

void Rectangle::setFront(float front)
{
    origin_(0) = front;
    setDepth( front - back());
}


bool Rectangle::empty() const
{
    return (width() <= 0) || (height() <= 0) || (depth() <= 0 ) || (volume() <=0);
}

void Rectangle::setVolume(float volume)
{
    volume_ = volume;
}

float Rectangle::volume() const
{
    //max() requires that the first and second arguments are of the same type
    return width() * height() * depth();
}


Rectangle Rectangle::changeToPclCoordinateSystem() const{
    Eigen::Vector3f pclOrigin( diagonal()(0), diagonal()(1), origin()(2));
    Rectangle rec( pclOrigin, depth(), height(), width(), resolution());
    rec.setDiagonal(Eigen::Vector3f( origin()(0), origin()(1), diagonal()(2)));
    return rec;
}

//TODO use ratio instead of volume
bool Rectangle::operator<(const Rectangle & rect){
    return volume() < rect.volume();
}

ostream & FFLD::operator<<(ostream & os, const Rectangle & rect)
{
    return os << rect.origin()(0) << ' ' << rect.origin()(1) << ' ' << rect.origin()(2)
              <<' ' << rect.depth() << ' ' << rect.height() << ' ' << rect.width() << ' ' << rect.resolution();
}

istream & FFLD::operator>>(istream & is, Rectangle & rect)
{
    float x, y, z, width, height, depth;
    float resolution;
	
    is >> z >> y >> x >> depth >> height >> width >> resolution;
	
    rect.setOrigin(Eigen::Vector3f( z, y, x));
    rect.setDiagonal(Eigen::Vector3f( z+depth, y+height, x+width));
    rect.setWidth(width);
	rect.setHeight(height);
    rect.setDepth(depth);
    rect.setResolution(resolution);
    rect.setVolume( rect.volume());
	
	return is;
}
