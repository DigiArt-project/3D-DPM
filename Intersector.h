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

#ifndef FFLD_INTERSECTOR_H
#define FFLD_INTERSECTOR_H

#include "Rectangle.h"

#include <algorithm>

namespace FFLD
{
/// Functor used to test for the intersection of two rectangles according to the Pascal criterion
/// (area of intersection over area of union).
class Intersector
{
public:
	/// Constructor.
	/// @param[in] reference The reference rectangle.
	/// @param[in] threshold The threshold of the criterion.
	/// @param[in] felzenszwalb Use Felzenszwalb's criterion instead of the Pascal one (area of
	/// intersection over area of second rectangle). Useful to remove small detections inside bigger
	/// ones.
	Intersector(const Rectangle & reference, double threshold = 0.5, bool felzenszwalb = false) :
	reference_(reference), threshold_(threshold), felzenszwalb_(felzenszwalb)
	{
	}
	
	/// Tests for the intersection between a given rectangle and the reference.
	/// @param[in] rect The rectangle to intersect with the reference.
	/// @param[out] score The score of the intersection.
	bool operator()(const Rectangle & rect, double * score = 0) const
	{
        if (score)
            *score = 0.0;
//		const int topFrontLeftX = std::max(reference_.topFrontLeft().x(), rect.topFrontLeft().x());
//        const int topFrontRightX = std::min(reference_.topFrontRight().x(), rect.topFrontRight().x());
        
//        if (topFrontRightX < topFrontLeftX)
//            return false;
        
//        const int topFrontLeftY = std::max(reference_.topFrontLeft().y(), rect.topFrontLeft().y());
//        const int bottomFrontLeftY = std::min(reference_.bottomFrontLeft().y(), rect.bottomFrontLeft().y());
        
//        if (bottomFrontLeftY < topFrontLeftY)
//            return false;
        
//        const int topFrontLeftZ = std::max(reference_.topFrontLeft().z(), rect.topFrontLeft().z());
//        const int topBackLeftZ = std::min(reference_.topBackLeft().z(), rect.bottomBackLeft().z());
        
//        if (topBackLeftZ < topFrontLeftZ)
//            return false;

        const int left = std::max(reference_.left(), rect.left());
        const int right = std::min(reference_.right(), rect.right());

        if (right < left)
            return false;

        const int top = std::max(reference_.top(), rect.top());
        const int bottom = std::min(reference_.bottom(), rect.bottom());

        if (bottom < top)
            return false;

        const int front = std::max(reference_.front(), rect.front());
        const int back = std::min(reference_.back(), rect.back());

        if (back < front)
            return false;
        
        const int intersectionVolume = (right - left + 1) * (bottom - top + 1) * (back - front + 1);
        const int cubeVolume = rect.volume();
        
        if (felzenszwalb_) {
            if (intersectionVolume >= cubeVolume * threshold_) {
                if (score)
                    *score = static_cast<double>(intersectionVolume) / cubeVolume;
                
                return true;
            }
        }
        else {
            const int referenceVolume = reference_.volume();
            const int unionVolume = referenceVolume + cubeVolume - intersectionVolume;
            
            if (intersectionVolume >= unionVolume * threshold_) {
                if (score)
                    *score = static_cast<double>(intersectionVolume) / unionVolume;
                
                return true;
            }
        }
		
		return false;
	}
	
private:
	Rectangle reference_;
	double threshold_;
	bool felzenszwalb_;
};
}

#endif
