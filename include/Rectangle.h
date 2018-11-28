#ifndef FFLD_RECTANGLE_H
#define FFLD_RECTANGLE_H

#include <iosfwd>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/convex_hull.h>

//Other
#include "typedefs.h"

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
//    Rectangle(float depth, float height, float width, float resolution);
	
	/// Constructs a rectangle with coordinates (@p x, @p y) and the given @p width and @p height.
    Rectangle(Eigen::Vector3f origin, Eigen::Vector3f boxSizes, Eigen::Matrix4f tform = Eigen::Matrix4f::Identity());

//    virtual ~Rectangle();

    ///Correspond to the top left corner of the rectangle
	/// Returns the x-coordinate of the rectangle.
    Eigen::Vector3f origin() const;

    Eigen::Vector3f size() const;

    float origin( int i) const;

    float size( int i) const;

    PointCloudT cloud() const;

    PointType cloud( int index) const;

//    void setCloud( PointCloudPtr cloud);

    Eigen::Matrix4f transform() const;
	
    /// Returns whether the rectangle is empty. An empty rectangle has no volume.
    bool empty() const;
    
    /// Returns the volume of the rectangle.
    /// @note Equivalent to max(width(), 0) * max(height(), 0)* max(depth(), 0).
    float volume() const;
    
    /// compare volume
    bool operator<(const Rectangle & rect) const;

protected:

    Eigen::Vector3f origin_;
    Eigen::Vector3f boxSizes_;
    PointCloudT cloud_;
    Eigen::Matrix4f tform_;
    float volume_;
};

/// Serializes a rectangle to a stream.
std::ostream & operator<<(std::ostream & os, const Rectangle & rect);

/// Unserializes a rectangle from a stream.
std::istream & operator>>(std::istream & is, Rectangle & rect);
}

#endif
