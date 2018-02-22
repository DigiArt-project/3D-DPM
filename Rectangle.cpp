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

#include <algorithm>
#include <iostream>

using namespace FFLD;
using namespace std;

Rectangle::Rectangle() : x_(0, 0, 0), y_(0, 0, 0), width_(0), height_(0), depth_(0)
{
}

Rectangle::Rectangle(float width, float height, float depth) :
    x_(0, 0, 0), y_(0, 0, 0), width_(width), height_(height), depth_(depth_)
{
}

Rectangle::Rectangle(Eigen::Vector3f x, Eigen::Vector3f y) :
    x_(x), y_(y), width_(0), height_(0), depth_(0)
{
    width_ = y.x - x.x;
    height_ = y.y - x.y;
    depth_ = y.z - x.z;
}

float Rectangle::x() const
{
	return x_;
}

void Rectangle::setX(float x)
{
	x_ = x;
}

float Rectangle::y() const
{
	return y_;
}

void Rectangle::setY(float y)
{
	y_ = y;
}

float Rectangle::z() const
{
    return z_;
}

void Rectangle::setZ(float z)
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
    return x().x;
}

void Rectangle::setLeft(Eigen::Vector3f left)
{
    setWidth(right() - left.x);
	setX(left);
}

float Rectangle::top() const
{
    return y().y;
}

void Rectangle::setTop(Eigen::Vector3f top)
{
    setHeight(top.y - bottom());
	setY(top);
}

float Rectangle::right() const
{
    return x().x + width();
}

void Rectangle::setRight(Eigen::Vector3f right)
{
    setWidth(right.x - left());
}

float Rectangle::bottom() const
{
    return x().y;
}

void Rectangle::setBottom(Eigen::Vector3f bottom)
{
    setHeight(top() - bottom.y);
}

float Rectangle::front() const
{
    return x().z;
}

void Rectangle::setFront(Eigen::Vector3f front)
{
    setDepth(back() - front.z);
}

float Rectangle::back() const
{
    return y().z;
}

void Rectangle::setBack(Eigen::Vector3f back)
{
    setDepth(back.z - front());
}

bool Rectangle::empty() const
{
    return (width() <= 0) || (height() <= 0) || (depth() <= 0);
}

float Rectangle::volume() const
{
    return max(width(), 0) * max(height(), 0) * max(depth(), 0);
}

ostream & FFLD::operator<<(ostream & os, const Rectangle & rect)
{
    return os << rect.x() << ' ' << rect.y() << ' ' << rect.width() << ' ' << rect.height() << ' ' << rect.depth();
}

istream & FFLD::operator>>(istream & is, Rectangle & rect)
{
    Eigen::Vector3f x, y;
    float width, height, depth;
	
    is >> x >> y >> width >> height >> depth;
	
    rect.setX(x);
	rect.setY(y);
	rect.setWidth(width);
	rect.setHeight(height);
    rect.setDepth(depth);
	
	return is;
}
