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
    Rectangle(float depth, float height, float width, float resolution);
	
	/// Constructs a rectangle with coordinates (@p x, @p y) and the given @p width and @p height.
    Rectangle(Eigen::Vector3f origin, float depth, float height, float width, float resolution);
	
    ///Correspond to the top left corner of the rectangle
	/// Returns the x-coordinate of the rectangle.
    Eigen::Vector3f origin() const;

    Eigen::Vector3f diagonal() const;
	
	/// Sets the x coordinate of the rectangle to @p x.
    void setOrigin(Eigen::Vector3f origin);

    void setDiagonal(Eigen::Vector3f diagonal);


	/// Returns the width of the rectangle.
    float width() const;
	/// Sets the height of the rectangle to the given @p width.
    void setWidth(float width);
	
	/// Returns the height of the rectangle.
    float height() const;
	/// Sets the height of the rectangle to the given @p height.
    void setHeight(float height);
    
    /// Returns the depth of the rectangle
    float depth() const ;
    /// Sets the depth of the rectangle to the given @p depth.
    void setDepth(float depth);
    
    /// Returns whether the rectangle is empty. An empty rectangle has no volume.
    bool empty() const;
    
    /// Returns the volume of the rectangle.
    /// @note Equivalent to max(width(), 0) * max(height(), 0)* max(depth(), 0).
    float volume() const;
    void setVolume(float volume);

    float right() const;
    float left() const;
    float top() const;
    float bottom() const;
    float front() const;
    float back() const;

    float resolution() const;
    void setResolution( float resolution);


    void setLeft(float left);
    void setRight(float right);
    void setBottom(float bottom);
    void setTop(float top);
    void setBack(float back);
    void setFront(float front);
    
    Rectangle changeToPclCoordinateSystem() const;

    /// compare ratio
    bool operator<(const Rectangle & rect);

private:

    Eigen::Vector3f origin_;
    Eigen::Vector3f diagonal_;
//    Eigen::Quaternion quaternion_;
    float width_;
    float height_;
    float depth_;
    float volume_;
    float resolution_;//become useless
};

/// Serializes a rectangle to a stream.
std::ostream & operator<<(std::ostream & os, const Rectangle & rect);

/// Unserializes a rectangle from a stream.
std::istream & operator>>(std::istream & is, Rectangle & rect);
}

#endif
