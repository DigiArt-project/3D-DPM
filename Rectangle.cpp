//--------------------------------------------------------------------------------------------------
// Implementation of the papers "Exact Acceleration of Linear Object Detectors", 12th European
// Conference on Computer Vision, 2012 and "Deformable Part Models with Individual Part Scaling",
// 24th British Machine Vision Conference, 2013.
//
// Copyright (c) 2013 Idiap Research Institute, <http://www.idiap.ch/>
// Written by Charles Dubout <charles.dubout@idiap.ch>
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

Rectangle::Rectangle() : x_(0), y_(0),z_(0), width_(0), height_(0), depth_(0)
{
}
Rectangle::Rectangle(const Rectangle& rect){
    width_ = rect.width_;
    height_ = rect.height_;
    depth_ = rect.depth_;
    volume_ = rect.volume_;
}

Rectangle::Rectangle(float width, float height, float depth, float volume) :
    x_(0), y_(0),z_(0), width_(width), height_(height), depth_(depth), volume_(volume)
{
}

Rectangle::Rectangle(int x, int y, int z,int width, int height, int depth) :
    x_(x), y_(y),z_(z), width_(width), height_(height), depth_(depth)
{
    volume_ = this->volume();
}

int Rectangle::x() const
{
	return x_;
}

void Rectangle::setX(int x)
{
	x_ = x;
}

int Rectangle::y() const
{
	return y_;
}

void Rectangle::setY(int y)
{
	y_ = y;
}

int Rectangle::z() const
{
    return z_;
}

void Rectangle::setZ(int z)
{
    z_ = z;
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

float Rectangle::left() const
{
    return x();
}

void Rectangle::setLeft(int left)
{
    setWidth(right() - left + 1);
	setX(left);
}

float Rectangle::top() const
{
    return y();
}

void Rectangle::setTop(int top)
{
    setHeight(bottom() - top + 1);
	setY(top);
}

float Rectangle::right() const
{
    return x() + width();
}

void Rectangle::setRight(int right)
{
    setWidth(right - left());
}

float Rectangle::bottom() const
{
    return y() + height() - 1;
}

void Rectangle::setBottom(int bottom)
{
    setHeight(top() - bottom);
}

float Rectangle::backBottom() const{
    return bottom() + depth() - 1;
}

void Rectangle::setBackBottom(int backB){
    setDepth(backBottom() - backB);
}
float Rectangle::backTop() const
{
    return (top() + depth() - 1);
}

void Rectangle::setBackTop(int backT)
{
    setDepth(backTop() - backT);
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
    return std::max(width(), float(0)) * std::max(height(), float(0)) * std::max(depth(), float(0));
}

ostream & FFLD::operator<<(ostream & os, const Rectangle & rect)
{
    return os << rect.x() << ' ' << rect.y() << ' ' << rect.z() <<' ' << rect.width() << ' ' << rect.height() << ' ' << rect.depth();
}

istream & FFLD::operator>>(istream & is, Rectangle & rect)
{
    int x, y, z;
    float width, height, depth, volume;
	
    is >> x >> y >> z >> width >> height >> depth >> volume;
	
    rect.setX(x);
	rect.setY(y);
    rect.setZ(z);
    rect.setWidth(width);
	rect.setHeight(height);
    rect.setDepth(depth);
    rect.setVolume(volume);
	
	return is;
}
