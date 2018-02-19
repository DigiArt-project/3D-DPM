
#ifndef FFLD_POINTCLOUD_H
#define FFLD_POINTCLOUD_H

#include <iosfwd>
#include <string>
#include <vector>
#include <stdint.h>

namespace FFLD
{

class PointCloudData
{
public:

    PointCloudData();
	
	/// Constructs an image with the given @p width, @p height and @p depth, and initializes it from
	/// the given @p bits.
	/// @note The returned image might be empty if any of the parameters is incorrect.
	PointCloudData(int width, int height, int depth, const uint8_t * bits = 0);
	
	/// Constructs an image and tries to load the image from the jpeg file with the given
	/// @p filename.
	/// @note The returned image might be empty if the image could not be loaded.
	PointCloudData(const std::string & filename);
	
	/// Returns whether the image is empty. An empty image has zero size.
	bool empty() const;
	
	/// Returns the width of the image.
	int width() const;
	
	/// Returns the height of the image.
	int height() const;
	
	/// Returns the depth of the image. The image depth is the number of color channels.
	int depth() const;
	
	/// Returns a pointer to the pixel data. Returns a null pointer if the image is empty.
	const uint8_t * bits() const;
	
	/// Returns a pointer to the pixel data. Returns a null pointer if the image is empty.
	uint8_t * bits();
	
	/// Returns a pointer to the pixel data at the scanline with index y. The first scanline is at
	/// index 0. Returns a null pointer if the image is empty or if y is out of bounds.
	const uint8_t * scanLine(int y) const;
	
	/// Returns a pointer to the pixel data at the scanline with index y. The first scanline is at
	/// index 0. Returns a null pointer if the image is empty or if y is out of bounds.
	uint8_t * scanLine(int y);
	
	/// Saves the image to a jpeg file with the given @p filename and @p quality.
	void save(const std::string & filename, int quality = 100) const;
	
	/// Returns a copy of the image scaled to @scale. If the scale is zero or negative, the method
	/// returns an empty image.
	PointCloudData rescale(double scale) const;
	
private:
	int width_;
	int height_;
	int depth_;
	std::vector<uint8_t> bits_;
};

/// Serializes an image to a stream.
std::ostream & operator<<(std::ostream & os, const PointCloudData & image);

/// Unserializes an image from a stream.
std::istream & operator>>(std::istream & is, PointCloudData & image);
}

#endif
