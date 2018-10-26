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

#ifndef FFLD_SCENE_H
#define FFLD_SCENE_H

#include "Object.h"

#include <string>
#include <vector>
//#include "typedefs.h"

#include <libxml/parser.h>

namespace FFLD
{
/// The Scene class represents a Pascal scene, consisting of a (filename to a) jpeg image and a list
/// of Pascal objects. It stores most of the information present in a Pascal VOC 2007 .xml
/// annotation file. Missing are the <source>, <owner>, and <segmented> fields, as they are
/// irrelevant to the training or testing of object detectors. The <folder> and (image) <filename>
/// fields are also merged together into an absolute filename, derived from the scene filename.
class Scene
{
public:
	/// Constructs an empty scene. An empty scene has an empty image and no object.
	Scene();
	
	/// Constructs a scene from informations about a jpeg image and a list of objects.
    /// @param[in] width Width of the PC.
    /// @param[in] height Height of the PC.
    /// @param[in] depth Depth of the PC.
    /// @param[in] filename Filename of the PC.
	/// @param[in] objects List of objects present in the scene.
    Scene(/*const Eigen::Vector3i origin, */const std::string & filename,
          const std::vector<Object> & objects);
	
	/// Constructs a scene and tries to load the scene from the xml file with the given @p filename.
    /*explicit*/ Scene(const std::string & xmlName, const std::string & pcFileName, const float resolution);
	
	/// Returns whether the scene is empty. An empty scene has an empty image and no object.
	bool empty() const;
	
//    /// Returns the origin of the PC.
//    Eigen::Vector3i origin() const;

//    /// Sets the origin of the PC.
//    void setOrigin(Eigen::Vector3i origin);

//    /// Returns the width of the PC.
//	int width() const;
	
//    /// Sets the width of the PC.
//	void setWidth(int width);
	
//    /// Returns the height of the PC.
//	int height() const;
	
//    /// Sets the height of the PC.
//	void setHeight(int height);
	
//    /// Returns the depth of the PC.
//	int depth() const;
	
//    /// Sets the depth of the PC.
//	void setDepth(int depth);
	
    /// Returns the filename of the PC.
	const std::string & filename() const;
	
    /// Sets the filename of the PC.
	void setFilename(const std::string & filename);
	
	/// Returns the list of objects present in the scene.
	const std::vector<Object> & objects() const;
	
	/// Sets the list of objects present in the scene.
	void setObjects(const std::vector<Object> & objects);

    /// Returns the list of objects present in the scene.
    const std::vector<Eigen::Vector3f> & localPose() const;

    float resolution() const;
	
private:
//    Eigen::Vector3i origin_;
//	int width_;
//	int height_;
//	int depth_;
//    Eigen::Vector3i origin_;
//    Eigen::Vector3i size_;
    float resolution_;
    std::string pcFileName_;
	std::vector<Object> objects_;
    std::vector<Eigen::Vector3f> localPose_;
};

/// Serializes a scene to a stream.
std::ostream & operator<<(std::ostream & os, const Scene & scene);

/// Unserializes a scene from a stream.
std::istream & operator>>(std::istream & is, Scene & scene);
}

#endif
