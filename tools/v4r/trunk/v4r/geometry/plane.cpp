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

#include "plane.h"

using namespace V4R;
PlaneHdl::PlaneHdl()
        : mpPlane(NULL) {
};
PlaneHdl::PlaneHdl(Plane<double> &rPlane)
        : mpPlane(NULL)
{
    set(rPlane);
};

void PlaneHdl::set(Plane<double> &rPlane) {
    mpPlane = &rPlane;
    mP = cv::Mat_<double>(3,1,rPlane.p.val);
    mN = cv::Mat_<double>(3,1,rPlane.n.val);
    mEq = cv::Mat_<double>(3,1,rPlane.eq.val);
};

void PlaneHdl::computeNormal(const cv::Vec3d &p1, const cv::Vec3d &p2, const cv::Vec3d &p3) {
    cv::Vec<double, 3> d2 = p2 - p1;
    cv::Vec<double, 3> d3 = p3 - p1;
		mpPlane->n = d2.cross(d3);
		/*
    std::cout << "p1 = " << p1 << std::endl;
    std::cout << "p2 = " << p2 << std::endl;
    std::cout << "p3 = " << p3 << std::endl;
    std::cout << "d2 = " << d2 << std::endl;
    std::cout << "d3 = " << d3 << std::endl;
    std::cout << "n = " << mpPlane->n << std::endl;
    */
		cv::normalize(mN, mN);
    //std::cout << "n = " << mpPlane->n << std::endl;
};
