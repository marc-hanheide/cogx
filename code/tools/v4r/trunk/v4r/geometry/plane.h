/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) <year>  <name of author>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef PLANE_H
#define PLANE_H

#include <v4r/geometry/structs.h>

namespace V4R {
class PlaneHdl
{
public:
    PlaneHdl();
    PlaneHdl(Plane<double> &rPlane);
		Plane<double> &operator ()(){
			return *mpPlane;
		};
		void set(Plane<double> &rPlane);
		void computeNormal(const cv::Vec3d &p1, const cv::Vec3d &p2, const cv::Vec3d &p3);
private:
    Plane<double> *mpPlane;
		cv::Mat_<double> mN;
		cv::Mat_<double> mEq;
		cv::Mat_<double> mP;
};
};
#endif // PLANE_H
