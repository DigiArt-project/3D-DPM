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

Rectangle::Rectangle() : origin_( 0, 0, 0), diagonal_( 0, 0, 0), width_(0), height_(0), depth_(0)
{
}
Rectangle::Rectangle(const Rectangle& rect)
    : origin_( rect.origin_), width_(rect.width_), height_(rect.height_), depth_(rect.depth_),
      diagonal_( rect.width_, rect.height_, rect.depth_), volume_(rect.volume_)
{}

Rectangle::Rectangle(int width, int height, int depth) :
    origin_( 0, 0, 0), width_(width), height_(height), depth_(depth),
    diagonal_( width_, height_, depth_)
{
    volume_ = volume();
}

Rectangle::Rectangle(Eigen::Vector3i origin, int width, int height, int depth) :
    origin_( origin), width_(width), height_(height), depth_(depth),
    diagonal_( origin.x() + width_, origin.y() + height_, origin.z() + depth_)
{
    volume_ = volume();
}

Eigen::Vector3i Rectangle::origin() const
{
    return origin_;
}

Eigen::Vector3i Rectangle::diagonal() const
{
    return diagonal_;
}

void Rectangle::setOrigin(Eigen::Vector3i origin)
{
    origin_ = origin;
}

void Rectangle::setDiagonal(Eigen::Vector3i diagonal)
{
    diagonal_ = diagonal;
}

int Rectangle::width() const
{
	return width_;
}

void Rectangle::setWidth(int width)
{
	width_ = width;
}

int Rectangle::height() const
{
	return height_;
}

void Rectangle::setHeight(int height)
{
	height_ = height;
}

int Rectangle::depth() const
{
    return depth_;
}

void Rectangle::setDepth(int depth)
{
    depth_ = depth;
}

int Rectangle::right() const
{
    return diagonal_.x();
}

int Rectangle::left() const
{
    return origin_.x();
}

int Rectangle::top() const
{
    return origin_.y();
}

int Rectangle::bottom() const
{
    return diagonal_.y();
}

int Rectangle::front() const
{
    return origin_.z();
}

int Rectangle::back() const
{
    return diagonal_.z();
}


void Rectangle::setLeft(int left)
{
//    if( left > 0 && left < right()){
        origin_.x() = left;
        setWidth( right() - left);
//    } else{
//        cerr << "Try to set wrong Rectangle parameter : setLeft()" << endl;
//    }
}

void Rectangle::setRight(int right)
{
//    if( right > left()){
        diagonal_.x() = right;
        setWidth( right - left());
//    } else{
//        cerr << "Try to set wrong Rectangle parameter : setRight()" << endl;
//    }
}

void Rectangle::setBottom(int bottom)
{
//    if( bottom > top()){
        diagonal_.y() = bottom;
        setHeight( top() - bottom);
//    } else{
//        cerr << "Try to set wrong Rectangle parameter : setBottom()" << endl;
//    }
}

void Rectangle::setTop(int top)
{
//    if( top > 0 && top < bottom()){
        origin_.y() = top;
        setHeight( top - bottom());
//    } else{
//        cerr << "Try to set wrong Rectangle parameter : setTop()" << endl;
//    }
}

void Rectangle::setBack(int back)
{
//    if( back > front()){
        diagonal_.z() = back;
        setDepth( front() - back);
//    } else{
//        cerr << "Try to set wrong Rectangle parameter : setBack()" << endl;
//    }
}

void Rectangle::setFront(int front)
{
//    if( front > 0 && front < back()){
        origin_.z() = front;
        setDepth( front - back());
//    } else{
//        cerr << "Try to set wrong Rectangle parameter : setFront()" << endl;
//    }
}

//Eigen::Vector3f Rectangle::topFrontLeft() const{
//    //Eigen::Vector3f topFrontLeft(this->x_,this->y_,this->z_);
//    return this->topFrontLeft_;
//}

//Eigen::Vector3f Rectangle::topFrontRight() const{
//    //Eigen::Vector3f topFrontRight(this->x_ + this->width(),this->y_,this->z_);
//    return this->topFrontRight_;
//}
//Eigen::Vector3f Rectangle::bottomFrontLeft() const{
//    //Eigen::Vector3f bottomFrontLeft(this->x_,this->y_ + this->height(),this->z_);
//    return this->bottomFrontLeft_;
//}
//Eigen::Vector3f Rectangle::bottomFrontRight() const{
//    //Eigen::Vector3f bottomFrontRight(this->x_ + this->width(),this->y_ + this->height(),this->z_);
//    return this->bottomFrontRight_;
//}
//Eigen::Vector3f Rectangle::topBackLeft() const{
//    //Eigen::Vector3f topBackLeft(this->x_ ,this->y_ ,this->z_ + this->depth());
//    return this->topBackLeft_;
//}
//Eigen::Vector3f Rectangle::topBackRight() const{
//    //Eigen::Vector3f topBackRight(this->x_ + this->width() ,this->y_ ,this->z_ + this->depth());
//    return this->topBackRight_;
//}
//Eigen::Vector3f Rectangle::bottomBackLeft() const{
//    //Eigen::Vector3f bottomBackLeft(this->x_ ,this->y_ + this->height(),this->z_ + this->depth());
//    return this->bottomBackLeft_;
//}
//Eigen::Vector3f Rectangle::bottomBackRight() const{
//    //Eigen::Vector3f bottomBackRight(this->x_ + this->width(), this->y_ + this->height(),this->z_ + this->depth());
//    return this->bottomBackRight_;
//}

//void Rectangle::setTopFrontLeft(Eigen::Vector3f pt){
//    this->topFrontLeft_ = pt;
//}
//void Rectangle::settopFrontRight(Eigen::Vector3f pt){
//    this->topFrontRight_ = pt;
//}
//void Rectangle::setTopBackLeft(Eigen::Vector3f pt){
//    this->topBackLeft_ = pt;
//}
//void Rectangle::setTopBackRight(Eigen::Vector3f pt){
//    this->topBackRight_ = pt;
//}

//void Rectangle::bottomFrontLeft(Eigen::Vector3f pt){
//    this->bottomFrontLeft_ = pt;
//}
//void Rectangle::bottomFrontRight(Eigen::Vector3f pt){
//    this->bottomFrontRight_ = pt;
//}
//void Rectangle::bottomBackLeft(Eigen::Vector3f pt){
//    this->bottomBackLeft_ = pt;
//}
//void Rectangle::bottomBackRight(Eigen::Vector3f pt){
//    this->bottomBackRight_ = pt;
//}


bool Rectangle::empty() const
{
    return (width() <= 0) || (height() <= 0) || (depth() <= 0 ) || (volume() <=0);
}

void Rectangle::setVolume(int volume)
{
    volume_ = volume;
}

int Rectangle::volume() const
{
    //max() requires that the first and second arguments are of the same type
    return width() * height() * depth();
}

//Eigen::Vector3f substractValue(Eigen::Vector3f refPoint, Eigen::Vector3f valueToSubstract){
//    Eigen::Vector3f newPointValue(refPoint.x() - valueToSubstract.x(), refPoint.y() - valueToSubstract.y(),refPoint.z() - valueToSubstract.z());
//    return newPointValue;
//}

Rectangle Rectangle::changeToPclCoordinateSystem() const{
    Eigen::Vector3i pclOrigin( origin().x(), diagonal().y(), diagonal().z());

    return Rectangle( pclOrigin, width(), height(), depth());
//    this->topFrontLeft_ = substractValue(this->topFrontLeft(),Eigen::Vector3f(0,-this->height(),0));
//    this->topFrontRight_ = substractValue(this->topFrontRight(),Eigen::Vector3f(0,-this->height(),0));
//    this->bottomFrontLeft_ = substractValue(this->bottomFrontLeft(),Eigen::Vector3f(0,this->height(),0));
//    this->bottomFrontRight_ = substractValue(this->bottomFrontRight(),Eigen::Vector3f(0,this->height(),0));
//    this->topBackLeft_ = substractValue(this->topBackLeft(),Eigen::Vector3f(0,-this->height(),2 * this->depth()));
//    this->topBackRight_ = substractValue(this->topBackRight(),Eigen::Vector3f(0,-this->height(),2 * this->depth()));
//    this->bottomBackLeft_ = substractValue(this->bottomBackLeft(),Eigen::Vector3f(0,this->height(),2 * this->depth()));
//    this->bottomBackRight_ = substractValue(this->bottomBackRight(),Eigen::Vector3f(0,this->height(),2 * this->depth()));
}

//void Rectangle::toString() const{
    
//    int xmax = x_ + this->width();
//    int ymax = y_ + this->height();
//    int zmax = z_ + this->depth();
    
//    std::cout<<"Cube specifications :"<<std::endl<<std::endl;
//    std::cout<<"(xmin,ymin,zmin) : "<< "(" << x_ << "," << y_ << "," << z_  << ")" <<std::endl;
//    std::cout<<"(xmax,ymax,zmax) : "<< "(" << xmax << "," << ymax << "," << zmax  << ")" <<std::endl;
//    std::cout<<"Width : "<<this->width()<<std::endl;
//    std::cout<<"Heigth : "<<this->height()<<std::endl;
//    std::cout<<"Depth : "<<this->depth()<<std::endl;
//    std::cout<<"Volume : "<<this->volume() <<std::endl;
//}

ostream & FFLD::operator<<(ostream & os, const Rectangle & rect)
{
    return os << rect.origin().x() << ' ' << rect.origin().y() << ' ' << rect.origin().z()
              <<' ' << rect.width() << ' ' << rect.height() << ' ' << rect.depth() << ' ' << rect.volume();
}

istream & FFLD::operator>>(istream & is, Rectangle & rect)
{
    int x, y, z, width, height, depth, volume;
	
    is >> x >> y >> z >> width >> height >> depth >> volume;
	
    rect.setOrigin(Eigen::Vector3i( x, y, z));
    rect.setDiagonal(Eigen::Vector3i( x+width, y+height, z+depth));
    rect.setWidth(width);
	rect.setHeight(height);
    rect.setDepth(depth);
    rect.setVolume(volume);
	
	return is;
}
