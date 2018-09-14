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

        const float left = std::max(reference_.left() /** reference_.resolution()*/, rect.left() /** rect.resolution()*/);
        const float right = std::min(reference_.right() /** reference_.resolution()*/, rect.right() /** rect.resolution()*/);

        if (right < left)
            return false;

        const float top = std::max(reference_.top() /** reference_.resolution()*/, rect.top() /** rect.resolution()*/);
        const float bottom = std::min(reference_.bottom() /** reference_.resolution()*/, rect.bottom() /** rect.resolution()*/);

        if (bottom < top)
            return false;

        const float front = std::max(reference_.front() /** reference_.resolution()*/, rect.front() /** rect.resolution()*/);
        const float back = std::min(reference_.back() /** reference_.resolution()*/, rect.back() /** rect.resolution()*/);

        if (back < front)
            return false;
        
        const float intersectionVolume = (right - left/* + 1*/) * (bottom - top/* + 1*/) * (back - front/* + 1*/);
        const float cubeVolume = rect.depth() * rect.width() * rect.height();//rect.volume();
        
        std::cout<<"intersectionVolume : "<<intersectionVolume<<std::endl;
        std::cout<<"cubeVolume : "<<cubeVolume<<std::endl;

        if (felzenszwalb_) {
            if (intersectionVolume >= cubeVolume * threshold_) {
                if (score)
                    *score = intersectionVolume / cubeVolume;
                
                return true;
            }
        }
        else {
            const float referenceVolume = reference_.depth() * reference_.width() * reference_.height();//reference_.volume();
            const float unionVolume = referenceVolume + cubeVolume - intersectionVolume;
            if (intersectionVolume >= unionVolume * threshold_) {
                if (score)
                    *score = intersectionVolume / unionVolume;
                
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
