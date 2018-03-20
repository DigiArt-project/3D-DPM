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

#ifndef FFLD_RECTANGLE_H
#define FFLD_RECTANGLE_H

#include <iosfwd>

#include <algorithm>
#include <iostream>
//EIGEN
#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

namespace FFLD
{
/// The Rectangle class defines a rectangle in the plane using floateger precision. If the coordinates
/// of the top left corner of the rectangle are (x, y), the coordinates of the bottom right corner
/// are (x + width - 1, y + height - 1), where width and height are the dimensions of the rectangle.
/// The corners are therefore understood as the extremal pofloats still inside the rectangle.
class Rectangle
{
public:
	/// Constructs an empty rectangle. An empty rectangle has no area.
	Rectangle();
    
    Rectangle(const Rectangle& rect);
	
	/// Constructs a rectangle with the given @p width and @p height.
    Rectangle(float width, float height, float depth,float volume);
	
	/// Constructs a rectangle with coordinates (@p x, @p y) and the given @p width and @p height.
    Rectangle(int x, int y, int z, int width, int height, int depth);
	
    ///Correspond to the top left corner of the rectangle
	/// Returns the x-coordinate of the rectangle.
    int x() const;
    
    /// Returns the y-coordinate of the rectangle.
    int y() const;
    
     /// Returns the z-coordinate of the rectangle.
    int z() const;
	
	/// Sets the x coordinate of the rectangle to @p x.
    void setX(int x);
    void setY(int y);
    void setZ(int z);


	/// Returns the width of the rectangle.
    float width() const;
	/// Sets the height of the rectangle to the given @p width.
    void setWidth(float width);
	
	/// Returns the height of the rectangle.
    float height() const;
	/// Sets the height of the rectangle to the given @p height.
    void setHeight(float height);
    
    //Return the depth of the rectangle
    float depth() const ;
    /// Sets the depth of the rectangle to the given @p depth.
    void setDepth(float depth);
    
    /// Returns whether the rectangle is empty. An empty rectangle has no volume.
    bool isEmpty() const;
    
    /// Returns the volume of the rectangle.
    /// @note Equivalent to max(width(), 0) * max(height(), 0)* max(depth(), 0).
    float volume() const;
    void setVolume(float volume);
    

    Eigen::Vector3f topFrontLeft() const;
    Eigen::Vector3f topFrontRight() const;
    Eigen::Vector3f topBackLeft() const;
    Eigen::Vector3f topBackRight() const;
    
    Eigen::Vector3f bottomFrontLeft() const;
    Eigen::Vector3f bottomFrontRight() const;
    Eigen::Vector3f bottomBackLeft() const;
    Eigen::Vector3f bottomBackRight() const;
    
    void setTopFrontLeft(Eigen::Vector3f pt);
    void settopFrontRight(Eigen::Vector3f pt);
    void setTopBackLeft(Eigen::Vector3f pt);
    void setTopBackRight(Eigen::Vector3f pt);
    
    void bottomFrontLeft(Eigen::Vector3f pt);
    void bottomFrontRight(Eigen::Vector3f pt);
    void bottomBackLeft(Eigen::Vector3f pt);
    void bottomBackRight(Eigen::Vector3f pt);
    
    void toString() const;
    
    void changeToPclCoordinateSystem();

	
private:
    int x_;
    int y_;
    int z_;
    Eigen::Vector3f topFrontLeft_;
    Eigen::Vector3f topFrontRight_;
    Eigen::Vector3f topBackLeft_;
    Eigen::Vector3f topBackRight_;
    Eigen::Vector3f bottomFrontLeft_;
    Eigen::Vector3f bottomFrontRight_;
    Eigen::Vector3f bottomBackLeft_;
    Eigen::Vector3f bottomBackRight_;
    float width_;
    float height_;
    float depth_;
    float volume_;
};

/// Serializes a rectangle to a stream.
std::ostream & operator<<(std::ostream & os, const Rectangle & rect);

/// Unserializes a rectangle from a stream.
std::istream & operator>>(std::istream & is, Rectangle & rect);
}

#endif
