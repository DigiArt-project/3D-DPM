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
    
    /// Sets the depth of the rectangle to the given @p depth.
    void setDepth(float depth);
    
    //Return the depth of the rectangle
    float depth() const ;
    
    /// Returns whether the rectangle is empty. An empty rectangle has no volume.
    bool empty() const;
    
    /// Returns the volume of the rectangle.
    /// @note Equivalent to max(width(), 0) * max(height(), 0)* max(depth(), 0).
    float volume() const;
    
    void setVolume(float volume);
    
	/// Returns the left side of the rectangle.
	/// @note Equivalent to x().
    float left() const;
	
	/// Sets the left side of the rectangle to @p left.
	/// @note The right side of the rectangle is not modified.
    void setLeft(int left);
	
	/// Returns the top side of the rectangle.
	/// @note Equivalent to y().
    float top() const;
	
	/// Sets the top side of the rectangle to @p top.
	/// @note The bottom side of the rectangle is not modified.
    void setTop(int top);
	
	/// Returns the right side of the rectangle.
	/// @note Equivalent to x() + width() - 1.
    float right() const;
	
	/// Sets the right side of the rectangle to @p right.
	/// @note The left side of the rectangle is not modified.
    void setRight(int right);
	
	/// Returns the bottom side of the rectangle.
	/// @note Equivalent to y() + height() - 1.
    float bottom() const;
	
	/// Sets the bottom side of the rectangle to @p bottom.
	/// @note The top side of the rectangle is not modified.
    void setBottom(int bottom);

    /// Returns the back top side of the rectangle.
    /// @note Equivalent to top() + depth() - 1.
    float backTop() const;
    /// Sets the back top side of the rectangle to @p backtop.
    void setBackTop(int backT);
    
    /// Returns the back bottom side of the rectangle.
    /// @note Equivalent to bottom() + depth() - 1.
    float backBottom() const;
    /// Sets the back top side of the rectangle to @p backtop.
    void setBackBottom(int backB);
    
  

	
private:
    int x_;
    int y_;
    int z_;
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
