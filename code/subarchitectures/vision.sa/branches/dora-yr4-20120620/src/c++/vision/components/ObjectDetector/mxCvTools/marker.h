/***************************************************************************
 *   Copyright (C) 2008 by Markus Bader   *
 *   bader@acin.tuwien.ac.at   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/**
 * @file marker.h
 * @author Markus Bader
 * @brief
 **/

#ifndef MARKER_H
#define MARKER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <stdio.h>
#include <algorithm>

/**
* @brief Image region or a blob
**/
class CImageRegion{
public:
	double dArea;       			/// occupied area in relative to the hole image area
	CvRect tBox;    					/// bounding box
	CvPoint2D64f tCenter;  		/// centroid
	CvScalar tYUVavr;    			/// average color
	CvScalar tBGRavr;    			/// average color
	inline bool operator< ( CImageRegion &rOther ){
		return (dArea < rOther.dArea);
	}
	inline bool operator<= ( CImageRegion &rOther ){
		return (dArea <= rOther.dArea);
	}
};

/**
* @brief Class which represents a detected marker (blob in 3D). It is used to create objects
**/
class CMarker{


public:
		CMarker():iIdTracking(-1) {}
		CvPoint3D64f tPosWorld;		/// location of the object in the world
		CImageRegion cImgRegion;	/// location of the object in the image
		int iIdTracking;					/// Tracking id
		inline bool operator<(CMarker &rOther ){
			return (cImgRegion < rOther.cImgRegion);
		}

		inline bool operator<= ( CMarker &rOther ){
			return (cImgRegion <= rOther.cImgRegion);
		}
};

/**
* @brief The model is a discribtion how a maker (blob in 3D) should look like
**/
class CMarkerModel{
public:
		std::string strName; /// name of the model
		CvScalar tColor; /// color to represent to marker
		double dAreaMin; /// min area of the pixels in the image related to one detected blob relative to the image size
		double dAreaMax; /// max area of the pixels in the image related to one detected blob relative to the image size
		double dAboveGround; /// location of the marker about the ground
		std::vector<CMarker> vMarker; /// detected markers

		/**
		* @brief sorts the detected marker by there size
		**/
		void Sort()
		{
			std::sort(vMarker.begin(), vMarker.end());
		}
};


inline bool operator<(const CMarker &a, const CMarker &b)
{
  return (a.cImgRegion.dArea < b.cImgRegion.dArea);
}

#endif
